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
use std::sync::{Arc, OnceLock};

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use nalgebra::Vector3;

use crate::ephemeris::{Ephemeris, EphemerisError, KM3_S2_TO_M3_S2};
use crate::forces::relativity::Relativity1PN;
use crate::forces::srp::SolarRadiationPressure;
use crate::forces::yarkovsky::YarkovskyA2;
use crate::forces::CompositeForce;
use crate::geometry::BPlaneEncounter;
use crate::perturber_field::{sb441_perturber_field, tier1_perturber_field, EphemerisPerturber};
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

/// Which Tier-2 force terms are enabled on the shipping field (HANDOFF §5/§6).
///
/// Every term is off by [`Default`], and that default is load-bearing: an all-off
/// config makes [`compose_force`] a [`CompositeForce`] holding the single Tier-1
/// point-mass term, whose per-evaluation acceleration is `0 + a_pointmass` — equal
/// to the bare `PointMassGravity` result to the last bit (`0.0 + x == x` in IEEE,
/// the sole exception `−0.0 → +0.0` being unobservable in any magnitude
/// downstream). So flipping Tier-2 in *without* enabling a term reproduces the
/// Tier-1 scenario's b-plane exactly — the "unchanged with them off" half of the
/// wiring's contract, checked empirically by the scenario tests.
///
/// Enabling a term makes the forward field disagree with the point-mass field the
/// seed was designed against, so the *same seed* now reaches a *different* b-plane
/// perigee — the "shifts with them on" half. That is measured, never asserted to a
/// hand-derived magnitude, by [`RealFieldScenario::nominal_encounter_with`].
/// Physical inputs for the solar-radiation-pressure term (HANDOFF §5), carried by
/// [`Tier2Config::srp`]. The cannonball model needs only the radiation-pressure
/// coefficient and the area-to-mass ratio; [`compose_force`] hands these to
/// [`SolarRadiationPressure::from_physical`], which folds in the solar constant and
/// `c`. A struct rather than a bare characteristic acceleration so the menu names
/// the physically meaningful knobs a body actually has.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SrpParams {
    /// Radiation-pressure coefficient `C_r`: 1 for a perfect absorber, up to 2 for
    /// a perfect reflector. ~1–1.5 for a real dark asteroid surface.
    pub cr: f64,
    /// Area-to-mass ratio `A/m`, m²/kg. A sub-km rock sits around 1e-6…1e-5;
    /// [`Self::sub_km_rock`] is a plausible default for the synthetic threat.
    pub area_to_mass_m2_per_kg: f64,
}

impl SrpParams {
    /// A plausible sub-km stony asteroid: a 300 m body (`r = 150 m`) at
    /// 2000 kg/m³ gives `A/m = 3/(4·r·ρ) ≈ 2.5e-6 m²/kg`, with `C_r = 1.3` for a
    /// dark, partly-reflecting surface. Yields `β ≈ 2.5e-9` — the physically tiny,
    /// un-amplified value the shipping toggle uses.
    pub fn sub_km_rock() -> Self {
        let (radius_m, density_kg_m3) = (150.0, 2000.0);
        Self {
            cr: 1.3,
            area_to_mass_m2_per_kg: 3.0 / (4.0 * radius_m * density_kg_m3),
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Tier2Config {
    /// Enable the 1PN relativistic Sun term (PPN Schwarzschild, β=γ=1). Its μ is
    /// taken from the *same* ANISE `SUN_J2000` GM the point-mass Sun uses, never a
    /// hardcoded constant, so GR and Newtonian gravity can never silently disagree
    /// on μ_sun. Over the default ~12 yr campaign this shifts the predicted b-plane
    /// perigee by a few hundred km — real, and still a hit (keyhole-precision
    /// territory, the reason GR matters for planetary defence).
    pub relativity: bool,
    /// Yarkovsky transverse `A2` (m/s² at 1 AU, JPL Sentry sign convention: `>0`
    /// prograde → outward secular drift, `<0` retrograde → inward), or `None` to
    /// disable. The shipping threat is synthetic, so any `A2` is *made up*; use a
    /// **physically plausible** value (~1e-13…1e-14 for a sub-km body) and report
    /// whatever b-plane shift it produces, even if sub-km. Do **not** amplify it to
    /// manufacture a visible shift — that is the display-grade lie this project
    /// keeps catching.
    pub yarkovsky_a2: Option<f64>,
    /// Enable the 16 sb441 main-belt asteroids as point-mass force perturbers
    /// ([`sb441_perturber_field`](crate::perturber_field::sb441_perturber_field)) —
    /// the belt bodies ASSIST integrates against, the residual floor the Tier-1
    /// capstone measured (HANDOFF §5). Requires the `sb441-n16.bsp` small-body
    /// kernel to be mounted on the scenario's ephemeris: [`RealFieldScenario::build`]
    /// mounts it when this is set, and [`build_with`](RealFieldScenario::build_with)
    /// requires the caller to have chained it on — either way [`compose_force`]
    /// fails loud if it is missing rather than silently dropping the perturbers.
    /// Over the default ~12 yr campaign the sixteen shift the predicted b-plane
    /// perigee by a small, measured amount (reported, never asserted to a magnitude).
    pub asteroid_perturbers: bool,
    /// Solar-radiation-pressure cannonball term ([`SrpParams`]), or `None` to
    /// disable. SRP is **radial** — it produces no secular along-track drift (that
    /// is Yarkovsky's role), only a small orbit-shape change — so its b-plane shift
    /// over the campaign is small (plausibly sub-km at a realistic `A/m`). Use a
    /// **physically plausible** [`SrpParams::sub_km_rock`] and report whatever shift
    /// it yields; do **not** inflate `A/m` to manufacture a visible one — the same
    /// display-grade lie the `yarkovsky_a2` note warns against.
    pub srp: Option<SrpParams>,
}

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
    /// Which Tier-2 force terms the field carries (HANDOFF §5/§6). [`Default`] is
    /// all-off, reproducing the Tier-1 scenario bit-for-bit; the back-propagation
    /// that designs the seed uses this same field, so a terms-on config yields a
    /// self-consistent (still-hitting) impactor rather than a broken one.
    pub tier2: Tier2Config,
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
            tier2: Tier2Config::default(),
        }
    }
}

/// Assemble the force field for a scenario: the Tier-1 point-mass field plus
/// whichever Tier-2 terms `tier2` enables, all summed in one [`CompositeForce`].
///
/// This is the single place the shipping field is constructed, so
/// [`RealFieldScenario::build_with`] (which designs and flies the seed) and
/// [`RealFieldScenario::nominal_encounter_with`] (which re-flies the *same* seed
/// through a differently-toggled field, to measure the shift) can never disagree
/// about what "GR on" or "Yarkovsky on" means.
///
/// The 1PN μ_sun and both terms' central body are drawn from the *same* `eph` and
/// the *same* `SUN_J2000` frame the Tier-1 field's Sun uses — so the relativistic
/// μ matches the Newtonian one, and the heliocentric `r`,`v` the terms difference
/// out is the Sun the point-mass gravity is already tracking.
fn compose_force(
    eph: &Arc<Ephemeris>,
    tier2: &Tier2Config,
) -> Result<CompositeForce, EphemerisError> {
    let point_mass = tier1_perturber_field(eph)?;
    let mut force = CompositeForce::new().with(Box::new(point_mass));

    if tier2.relativity {
        let mu_sun = eph.gm_km3_s2(SUN_J2000)? * KM3_S2_TO_M3_S2;
        let sun = EphemerisPerturber::new(Arc::clone(eph), SUN_J2000);
        force = force.with(Box::new(Relativity1PN::new(mu_sun, sun)));
    }
    if let Some(a2) = tier2.yarkovsky_a2 {
        let sun = EphemerisPerturber::new(Arc::clone(eph), SUN_J2000);
        force = force.with(Box::new(YarkovskyA2::standard(a2, sun)));
    }
    if tier2.asteroid_perturbers {
        // Fails loud if `eph` lacks the sb441 kernel — `build` mounts it when the
        // flag is set, and a `build_with` caller must have chained it on.
        force = force.with(Box::new(sb441_perturber_field(eph)?));
    }
    if let Some(p) = tier2.srp {
        let sun = EphemerisPerturber::new(Arc::clone(eph), SUN_J2000);
        force = force.with(Box::new(SolarRadiationPressure::from_physical(
            p.cr,
            p.area_to_mass_m2_per_kg,
            sun,
        )));
    }
    Ok(force)
}

/// Everything downstream failure mode of building/sweeping a scenario, unified so
/// every consumer (binary, egui app, gdext binding) surfaces one error type.
#[derive(Debug)]
pub enum ScenarioError {
    /// No DE kernel pair could be resolved — neither the environment nor any
    /// conventional directory had one. Carries
    /// [`kernels::not_found_message`](crate::kernels::not_found_message): every
    /// place searched plus how to fix it, because "kernels not found" alone
    /// sends the reader hunting through source for the search order.
    KernelsNotFound(String),
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
            ScenarioError::KernelsNotFound(detail) => {
                write!(
                    f,
                    "no DE kernel pair could be resolved\n{detail}"
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
    /// The full force field the seed is designed against and flown through: the
    /// Tier-1 point-mass sum plus any Tier-2 terms `cfg.tier2` enabled
    /// ([`compose_force`]). A [`CompositeForce`], not a bare `PointMassGravity`, so
    /// the realism ladder is expressed as *which terms are in this sum* (HANDOFF §5).
    force: CompositeForce,
    earth: EphemerisPerturber,
    mu_earth: f64,
    earth_radius: f64,
    scan: ScanOptions,

    epoch0: Epoch,
    seed: StateVector,
    impact_epoch: Epoch,
    cadence_seconds: f64,
    n_snapshots: u32,

    /// The nominal trajectory, propagated on first use and reused thereafter (see
    /// [`Self::nominal_clock`]). Not part of the built state: it is a *pure
    /// function* of `seed` + `force` + `epoch0`/cadence, all of which are fixed at
    /// construction, so caching it changes nothing about what this scenario means
    /// — only how often it is recomputed.
    nominal_cache: OnceLock<Clock>,

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
    ///
    /// **Empty when there is no deflection at all** — i.e.
    /// [`frame_from_arcs`](RealFieldScenario::frame_from_arcs) was called with
    /// `deflected: None`, the pre-plan picture whose only story is the nominal
    /// track spearing the disc. Empty is not the same as zero-length: a consumer
    /// draws *nothing*, not a point at the geocentre.
    pub deflected: Vec<Vector3<f64>>,
    /// Earth's solid-body radius, m (the disc to draw).
    pub earth_radius: f64,
    /// Gravitationally-focused capture radius at the nominal encounter, m — the
    /// effective collision cross-section; a perigee inside it is a hit.
    pub capture_radius: f64,
    /// Nominal b-plane perigee, m (≤ `capture_radius`: the hit being undone).
    pub nominal_perigee: f64,
    /// Deflected b-plane perigee, m, or `None` if the deflected pass left the scan
    /// gate entirely (a miss so wide it is off any sensible frame) — **or if there
    /// is no deflection**, which `deflected.is_empty()` is what distinguishes.
    ///
    /// The *best* outcome and the *absent* one therefore share a `None`, exactly as
    /// they share a `-1` at the Godot binding's FFI boundary. That collision is
    /// deliberate (there is genuinely no finite perigee in either case) and it is a
    /// trap: a consumer that wants the difference must ask for it, and one that
    /// treats `None` as failure reports a threat thrown clear of Earth as a hit.
    pub deflected_perigee: Option<f64>,
}

/// An already-flown deflected arc: the [`Clock`] and the [`BPlaneEncounter`] that
/// came out of the **same** [`DeflectionScenario::deflected_trajectory`] call.
///
/// The pair is one value on purpose. [`frame_from`](RealFieldScenario::frame_from)
/// guarantees that the deflected track it draws and the perigee that annotates it
/// come from a single propagation and so cannot disagree; it can guarantee that
/// because it does the propagation itself. Once that propagation moves out to the
/// caller — which is the whole point of
/// [`frame_from_arcs`](RealFieldScenario::frame_from_arcs), so a renderer holding a
/// freshly-flown arc does not fly it a second time — the guarantee is only as
/// strong as the caller keeping the two halves together. This type is what
/// "together" looks like: build it from one `deflected_trajectory` return and there
/// is no seam at which a track can acquire a foreign perigee.
#[derive(Debug, Clone, Copy)]
pub struct DeflectedArc<'a> {
    /// The post-impulse trajectory, covering `[deflection_epoch, span_end]`.
    pub clock: &'a Clock,
    /// The encounter that same propagation produced. `None` means the deflected
    /// pass left the scan gate — a miss so wide it is off any sensible frame, which
    /// is a *success*, not a missing value.
    pub encounter: Option<BPlaneEncounter>,
    /// The epoch the impulse was applied at. Samples earlier than this read the
    /// nominal track, since the impulse has not happened yet.
    pub deflection_epoch: Epoch,
}

impl RealFieldScenario {
    /// Load the DE440 field, design the impactor per `cfg`, back-propagate the
    /// seed, and verify the nominal reproduces a hit.
    ///
    /// Kernel paths come from [`kernels::resolve`](crate::kernels::resolve):
    /// `ASTEROID_DE_KERNEL` + `ASTEROID_PLANETARY_CONSTANTS` if exported, else a
    /// conventional directory. A caller that resolves paths itself (the Godot
    /// frontend, which cannot rely on a launched game inheriting either
    /// variable) uses [`build_with`](Self::build_with) instead.
    pub fn build(cfg: &ImpactorConfig) -> Result<Self, ScenarioError> {
        let k = crate::kernels::resolve()
            .ok_or_else(|| ScenarioError::KernelsNotFound(crate::kernels::not_found_message()))?;

        let mut eph = Ephemeris::load(&k.bsp)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            .with_constants(&k.pca)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        // The asteroid perturbers read positions from the sb441 small-body kernel;
        // chain it on here so `compose_force` finds it. Fail loud if the flag is set
        // but the (optional, 646 MB) kernel was not resolved alongside the DE pair —
        // an enabled-but-absent field is a wrong field, not a silently smaller one.
        if cfg.tier2.asteroid_perturbers {
            let sb = k.small_bodies.as_ref().ok_or_else(|| {
                ScenarioError::Ephemeris(
                    "asteroid perturbers enabled but no sb441-n16 small-body kernel was \
                     found next to the DE kernel"
                        .into(),
                )
            })?;
            eph = eph
                .with_constants(sb)
                .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        }
        Self::build_with(cfg, Arc::new(eph))
    }

    /// Build from an already-loaded ephemeris (the kernels resolved elsewhere).
    /// [`build`](Self::build) is the env-var convenience over this; a binding that
    /// loads the kernel itself (surfacing its own error to the UI) calls here.
    pub fn build_with(cfg: &ImpactorConfig, eph: Arc<Ephemeris>) -> Result<Self, ScenarioError> {
        let force = compose_force(&eph, &cfg.tier2)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
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
            nominal_cache: OnceLock::new(),
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
    ///
    /// **Cheap after the first call.** The nominal trajectory is propagated once
    /// and cached ([`Self::nominal_clock`]); this then only clones it. That makes
    /// a per-interaction caller (the planner re-evaluating on every nudge, which
    /// builds one of these each time) pay the multi-year cruise once for the whole
    /// session instead of once per keypress.
    pub fn deflection(&self) -> Result<DeflectionScenario<'_>, DeflectionError> {
        DeflectionScenario::with_nominal(
            Dop853::new(),
            &self.force,
            &self.earth,
            self.epoch0,
            self.nominal_clock()?.clone(),
            self.cadence_seconds,
            self.n_snapshots,
            self.scan,
            self.mu_earth,
            self.earth_radius,
        )
    }

    /// The nominal trajectory, propagated on first call and reused after.
    ///
    /// Safe to cache because it is fully determined by state fixed at build time
    /// (`seed`, `force`, `epoch0`, cadence, snapshot count) — the propagation is
    /// deterministic, so the cached clock is identical to a freshly flown one, and
    /// nothing here can hand back a nominal belonging to a different field.
    fn nominal_clock(&self) -> Result<&Clock, DeflectionError> {
        if let Some(clock) = self.nominal_cache.get() {
            return Ok(clock);
        }
        // Validate on the same terms `DeflectionScenario::new` would, so a bad
        // cadence is still an error here rather than an assert inside `propagate`.
        DeflectionScenario::validate(
            self.cadence_seconds,
            self.n_snapshots,
            self.mu_earth,
            self.earth_radius,
        )?;
        let nominal = Clock::propagate(
            &Dop853::new(),
            &self.force,
            self.epoch0,
            self.seed,
            self.cadence_seconds,
            self.n_snapshots,
        )?;
        // A racing thread may have filled it first; the value is deterministic, so
        // either clock is equally correct and the loser's is simply dropped.
        let _ = self.nominal_cache.set(nominal);
        Ok(self
            .nominal_cache
            .get()
            .expect("just set, or set by a racing thread"))
    }

    /// The campaign-start epoch (the seed's epoch).
    pub fn epoch0(&self) -> Epoch {
        self.epoch0
    }

    /// The impact epoch.
    pub fn impact_epoch(&self) -> Epoch {
        self.impact_epoch
    }

    /// Re-fly this scenario's **built seed** through the field with `tier2` terms
    /// toggled, and report the nominal Earth encounter it reaches — the direct
    /// measurement of *how much 1PN relativity / Yarkovsky moves the predicted
    /// impact* (HANDOFF §5/§6 wiring).
    ///
    /// The seed is held fixed — it is whatever [`build`](Self::build) designed
    /// (through *this* scenario's `cfg.tier2` field) — and only the forward force
    /// changes. That is the whole point: rebuilding with terms enabled would
    /// back-propagate the seed through the terms-on field too, reproducing the hit
    /// *by construction* and showing no shift at all. Fixing the seed and swapping
    /// only the field is what makes the perigee difference attributable to the
    /// terms rather than to a re-designed impactor.
    ///
    /// Passing `&Tier2Config::default()` (all-off) re-flies through the bare Tier-1
    /// field and returns the scenario's own baseline perigee to the last bit — the
    /// "unchanged with them off" invariant, callable as a self-check. Passing a
    /// terms-on config returns the shifted perigee; the caller takes the difference.
    /// `None` means the re-flown pass found no close approach inside the scan gate
    /// (a miss so wide it left the gate) — not an error, just no finite perigee.
    ///
    /// Cost: one full nominal propagation and one full-span scan, i.e. seconds. This
    /// is a measurement/what-if entry point, not something to call in a render loop.
    pub fn nominal_encounter_with(
        &self,
        tier2: &Tier2Config,
    ) -> Result<Option<BPlaneEncounter>, ScenarioError> {
        let force = compose_force(&self.ephemeris, tier2)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let nominal = Clock::propagate(
            &Dop853::new(),
            &force,
            self.epoch0,
            self.seed,
            self.cadence_seconds,
            self.n_snapshots,
        )
        .map_err(|e| ScenarioError::Integration(e.to_string()))?;
        let ds = DeflectionScenario::with_nominal(
            Dop853::new(),
            &force,
            &self.earth,
            self.epoch0,
            nominal,
            self.cadence_seconds,
            self.n_snapshots,
            self.scan,
            self.mu_earth,
            self.earth_radius,
        )?;
        Ok(ds.nominal_encounter()?)
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
        let (clock, encounter) = ds.deflected_trajectory(deflection_epoch, delta_v)?;
        self.frame_from_arcs(
            ds.nominal(),
            nominal_enc,
            Some(DeflectedArc {
                clock: &clock,
                encounter,
                deflection_epoch,
            }),
            half_window_seconds,
            n_samples,
        )
    }

    /// Sample an encounter frame from trajectories that have **already been flown**
    /// — the half of [`frame_from`](Self::frame_from) that does no propagation, and
    /// therefore the one a caller who already holds the arcs should use.
    ///
    /// `frame_from` is the convenience: it flies the deflected arc and delegates
    /// here. But a caller that has *just* flown that arc for its own purposes — the
    /// Godot binding's planner keeps the post-impulse `Clock` to answer position
    /// queries from — would pay for a second identical propagation by calling it.
    /// That is not a hypothetical cost: at this scenario's scale it is ~0.85 s
    /// against a ~0.35 s input debounce, i.e. the same "re-flying an arc nothing
    /// asked to be re-flown" defect the nominal cache exists to prevent, moved one
    /// level out.
    ///
    /// Pass `deflected: None` for the **pre-plan** picture: the nominal track and
    /// the numbers that annotate it, with no deflection anywhere. The resulting
    /// frame's `deflected` is *empty* (see the field docs — empty, not zero-length),
    /// and this path does no propagation whatsoever, only sampling — which is what
    /// lets a display show the incoming impact the instant the scenario is built,
    /// long before any plan exists.
    ///
    /// `nominal_clock` must be this scenario's nominal (from
    /// [`DeflectionScenario::nominal`], or the cached clone of it) and `nominal_enc`
    /// the encounter it produces ([`nominal_hit`](Self::nominal_hit)); they share
    /// this scenario's Earth source, which is what makes the geocentric transform
    /// below consistent with the b-plane numbers reported alongside it.
    pub fn frame_from_arcs(
        &self,
        nominal_clock: &Clock,
        nominal_enc: BPlaneEncounter,
        deflected: Option<DeflectedArc<'_>>,
        half_window_seconds: f64,
        n_samples: usize,
    ) -> Result<EncounterFrame, ScenarioError> {
        let n = n_samples.max(2);
        let center = self.impact_epoch.tdb_seconds_past_j2000();

        let mut sample_seconds = Vec::with_capacity(n);
        let mut nominal = Vec::with_capacity(n);
        let mut deflected_track = Vec::with_capacity(if deflected.is_some() { n } else { 0 });

        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            let t = center - half_window_seconds + 2.0 * half_window_seconds * frac;
            let epoch = Epoch::from_tdb_seconds_past_j2000(t);

            let earth_pos = self
                .earth
                .state_at(epoch)
                .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
                .position;

            let ast_nom = nominal_clock
                .state_at(epoch)
                .map_err(|e| ScenarioError::Integration(e.to_string()))?
                .position;

            if let Some(arc) = deflected {
                // Before the deflection epoch the asteroid is still on the nominal
                // track (the impulse has not happened yet); after it, read the
                // post-deflection clock. For the animation's near-impact window this
                // is always the post-deflection branch, but the guard keeps the
                // helper honest for a window that reaches back across the nudge —
                // and the arc's clock does not even *cover* the earlier epochs, so
                // without it this would be a lookup failure, not a wrong answer.
                let ast_defl = if t < arc.deflection_epoch.tdb_seconds_past_j2000() {
                    ast_nom
                } else {
                    arc.clock
                        .state_at(epoch)
                        .map_err(|e| ScenarioError::Integration(e.to_string()))?
                        .position
                };
                deflected_track.push(ast_defl - earth_pos);
            }

            sample_seconds.push(t);
            nominal.push(ast_nom - earth_pos);
        }

        Ok(EncounterFrame {
            sample_seconds,
            nominal,
            deflected: deflected_track,
            earth_radius: self.earth_radius,
            capture_radius: nominal_enc.capture_radius,
            nominal_perigee: nominal_enc.perigee,
            deflected_perigee: deflected.and_then(|a| a.encounter).map(|e| e.perigee),
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

    /// A built scenario must be able to **leave the thread that built it**, which
    /// is the entire point of the `Send` bounds on `ForceModel`/`PerturberEphemeris`
    /// /`GeocentricState`. Building is a ~10 s propagation: a frontend that cannot
    /// move it to a worker must freeze its display for those 10 s, so this is a
    /// UX-critical property of the core, not a Rust technicality.
    ///
    /// Kernel-free and compile-time — `assert_send` fails to *compile* if any of
    /// those bounds is dropped, which is a louder and cheaper failure than the
    /// frontend discovering it. `Arc<Ephemeris>` is asserted alongside because the
    /// worker builds from a clone of it while the main thread keeps serving planet
    /// positions from the same almanac; that requires `Sync`, which it has.
    #[test]
    fn a_built_scenario_and_its_ephemeris_can_cross_to_a_worker_thread() {
        fn assert_send<T: Send>() {}
        fn assert_sync<T: Sync>() {}
        assert_send::<RealFieldScenario>();
        assert_send::<Arc<Ephemeris>>();
        assert_sync::<Arc<Ephemeris>>();
        assert_send::<Clock>();
    }

    /// The nominal cache must be **invisible in the physics and decisive in the
    /// cost** — the two halves of the claim that justifies it.
    ///
    /// *Invisible*: the cached clock is compared against a fresh propagation from
    /// the same seed through the same field, and must agree **exactly** (same
    /// inputs, same deterministic code path — not "to a tolerance"). If those ever
    /// diverge, the cache is serving a trajectory the scenario would not fly, and
    /// every b-plane number downstream is quietly wrong.
    ///
    /// *Decisive*: `deflection()` used to call `DeflectionScenario::new`, which
    /// re-flew the whole multi-year cruise — ~10 s on this machine, paid **per
    /// call**, i.e. per planner nudge. It is now a clone of the cached clock. The
    /// threshold below is ~20× on either side of both outcomes, so it is a real
    /// regression tripwire rather than a flaky benchmark: a "tidy-up" back to
    /// `new()` fails here loudly instead of silently making the planner unusable.
    ///
    /// Kernel-gated; skips (does not fail) with no kernel.
    #[test]
    fn nominal_is_cached_identically_and_deflection_stops_re_flying_it() {
        if crate::kernels::resolve_for_test("nominal_is_cached_identically_*").is_none() {
            return;
        }

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");

        // `build` verifies its own round-trip through `deflection()`, so a built
        // scenario arrives with the nominal already flown — nothing downstream
        // should ever pay for it again.
        let cached = sc
            .nominal_cache
            .get()
            .expect("build's round-trip check should leave the nominal cached");

        // Invisible: identical to a fresh flight of the same seed in the same field.
        let fresh = Clock::propagate(
            &Dop853::new(),
            &sc.force,
            sc.epoch0,
            sc.seed,
            sc.cadence_seconds,
            sc.n_snapshots,
        )
        .expect("fresh nominal propagates");
        for epoch in [sc.epoch0, sc.impact_epoch] {
            let c = cached.state_at(epoch).expect("cached state");
            let f = fresh.state_at(epoch).expect("fresh state");
            assert_eq!(
                c.position, f.position,
                "cached nominal position differs from a fresh propagation at {epoch:?} — \
                 the cache is serving a trajectory this scenario would not fly"
            );
            assert_eq!(c.velocity, f.velocity, "cached nominal velocity differs");
        }

        // Decisive: building a DeflectionScenario is now a clone, not a cruise.
        let t = std::time::Instant::now();
        let ds = sc.deflection().expect("deflection builds");
        let elapsed = t.elapsed();
        assert!(
            elapsed < std::time::Duration::from_millis(500),
            "deflection() took {elapsed:?} — it is re-propagating the nominal again \
             (was ~10 s per call before the cache; the planner calls this per nudge)"
        );
        // …and the scenario it hands back still carries that same nominal.
        assert_eq!(
            ds.nominal()
                .state_at(sc.impact_epoch)
                .expect("state")
                .position,
            fresh.state_at(sc.impact_epoch).expect("state").position,
            "the DeflectionScenario's nominal is not the cached one"
        );
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
        if crate::kernels::resolve_for_test("propagate_free_matches_direct_step_in_the_field").is_none() {
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
        if crate::kernels::resolve_for_test("encounter_frame_track_agrees_with_reported_perigee").is_none() {
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

    /// The propagate/sample split must be a pure refactor, and the no-propagation
    /// half must actually not propagate.
    ///
    /// Two halves, and the first is the one with teeth: `frame_from_arcs` fed the
    /// arc that `frame_from` would have flown itself must return a **bit-identical**
    /// frame. Exact equality, not a tolerance — both walk the same epochs through
    /// the same dense output, so any difference at all means the split changed the
    /// physics rather than relocating it, and a tolerance would wave exactly that
    /// through. This is the assertion that lets the binding stop calling
    /// `frame_from` without anyone having to trust that the two agree.
    ///
    /// The second half pins the pre-plan picture (`deflected: None`): the nominal
    /// track and its numbers survive, the deflected track comes back **empty**
    /// rather than zeroed, and the perigee is `None`. An empty deflected track is
    /// how a renderer knows to draw nothing; a zero-length one would put a marker on
    /// Earth's centre, which is the "ZERO is a real place" failure this project
    /// keeps re-learning — here it would draw the asteroid at the geocentre, i.e. a
    /// direct hit, as the picture of *no plan yet*.
    ///
    /// Kernel-gated, like its neighbour.
    #[test]
    fn frame_from_arcs_matches_frame_from_and_draws_nothing_without_a_plan() {
        if crate::kernels::resolve_for_test("frame_from_arcs_matches_frame_from…").is_none() {
            return;
        }

        let cfg = ImpactorConfig::default();
        let sc = RealFieldScenario::build(&cfg).expect("scenario builds");
        let ds = sc.deflection().expect("deflection scenario");
        let nominal_enc = sc.nominal_hit(&ds).expect("nominal is a hit");

        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-sc.period_seconds);
        let seed = ds
            .nominal()
            .state_at(deflection_epoch)
            .expect("nominal state");
        let dv = 0.2 * along_track_unit(seed).expect("nominal has a heading");

        let half_window = ENCOUNTER_HALF_WINDOW_SECONDS;
        let n = ENCOUNTER_SAMPLES;

        // The convenience path: it flies the arc internally.
        let via_frame_from = sc
            .frame_from(&ds, nominal_enc, deflection_epoch, dv, half_window, n)
            .expect("frame_from");

        // The split path: fly the arc here (as the binding's planner does for its
        // own reasons) and hand the *pair* over — no second propagation.
        let (clock, encounter) = ds
            .deflected_trajectory(deflection_epoch, dv)
            .expect("deflected trajectory");
        let via_arcs = sc
            .frame_from_arcs(
                ds.nominal(),
                nominal_enc,
                Some(DeflectedArc {
                    clock: &clock,
                    encounter,
                    deflection_epoch,
                }),
                half_window,
                n,
            )
            .expect("frame_from_arcs");

        assert_eq!(
            via_arcs.sample_seconds, via_frame_from.sample_seconds,
            "split changed the sample epochs"
        );
        assert_eq!(
            via_arcs.nominal, via_frame_from.nominal,
            "split changed the nominal track"
        );
        assert_eq!(
            via_arcs.deflected, via_frame_from.deflected,
            "split changed the deflected track"
        );
        assert_eq!(
            via_arcs.deflected_perigee, via_frame_from.deflected_perigee,
            "split changed the reported deflected perigee"
        );
        assert_eq!(via_arcs.nominal_perigee, via_frame_from.nominal_perigee);
        assert_eq!(via_arcs.capture_radius, via_frame_from.capture_radius);
        assert_eq!(via_arcs.earth_radius, via_frame_from.earth_radius);

        // The pre-plan picture: nominal only, no propagation at all.
        let pre_plan = sc
            .frame_from_arcs(ds.nominal(), nominal_enc, None, half_window, n)
            .expect("frame_from_arcs with no deflection");

        assert!(
            pre_plan.deflected.is_empty(),
            "no plan must leave the deflected track EMPTY, got {} points",
            pre_plan.deflected.len()
        );
        assert_eq!(
            pre_plan.deflected_perigee, None,
            "no plan means no deflected perigee"
        );
        assert_eq!(
            pre_plan.nominal, via_frame_from.nominal,
            "the nominal track must not depend on whether a plan exists"
        );
        assert_eq!(pre_plan.nominal_perigee, via_frame_from.nominal_perigee);
        assert_eq!(pre_plan.capture_radius, via_frame_from.capture_radius);
    }

    /// Wiring the Tier-2 terms into the shipping field must satisfy both halves of
    /// its contract (HANDOFF §5/§6): the b-plane is **unchanged** when every term is
    /// off, and **shifts** by a resolvable, physically-sensible amount when 1PN
    /// relativity or Yarkovsky is on.
    ///
    /// The measurement holds the built seed fixed and re-flies it with terms toggled
    /// ([`RealFieldScenario::nominal_encounter_with`]). That is the *only* way a
    /// shift can appear: rebuilding with terms on would back-propagate the seed
    /// through the terms-on field, reproducing the hit by construction and showing
    /// nothing. Fixing the seed and changing only the forward force attributes the
    /// perigee move to the physics, not to a re-designed impactor.
    ///
    /// Assertions are **structural**, never hand-derived magnitudes:
    /// - *Off == baseline, bit-for-bit.* The all-off composite is `0 + a_pointmass`,
    ///   identical to the bare field; if it differs, the wiring perturbed the
    ///   shipping scenario, which it must not.
    /// - *GR shifts and still hits.* 1PN over the ~12 yr campaign moves the perigee
    ///   by a resolvable amount (hundreds of km at this geometry) yet keeps it inside
    ///   the capture radius — keyhole-precision territory. The magnitude is measured
    ///   and printed, not asserted to a number.
    /// - *Yarkovsky at a physical A2 shifts honestly.* `A2 = 1e-13 m/s²` (plausible
    ///   for a sub-km body, deliberately **not** amplified) moves the perigee by some
    ///   nonzero finite amount; whether that is km-scale or sub-km is reported, not
    ///   asserted large.
    ///
    /// Kernel-gated: skips (does not fail) with no kernel.
    #[test]
    fn tier2_terms_leave_the_bplane_unchanged_off_and_shift_it_on() {
        if crate::kernels::resolve_for_test("tier2_terms_…_shift_it_on").is_none() {
            return;
        }

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");

        // The shipping baseline: the nominal hit the default (all-off) scenario reports.
        let baseline = sc
            .deflection()
            .expect("deflection")
            .nominal_encounter()
            .expect("nominal reduces")
            .expect("nominal is a hit");

        // (a) Off == baseline, to the last bit.
        let off = sc
            .nominal_encounter_with(&Tier2Config::default())
            .expect("off re-fly")
            .expect("off pass is still an encounter");
        assert_eq!(
            off.perigee, baseline.perigee,
            "all-off Tier-2 re-fly must match the shipping perigee bit-for-bit"
        );

        // (b) GR on shifts the perigee by a resolvable amount and stays a hit.
        let gr = sc
            .nominal_encounter_with(&Tier2Config {
                relativity: true,
                yarkovsky_a2: None,
                ..Tier2Config::default()
            })
            .expect("GR re-fly")
            .expect("GR pass is still an encounter");
        let gr_shift = (gr.perigee - baseline.perigee).abs();
        println!(
            "1PN perigee shift over the campaign: {:.1} km \
             (baseline {:.1} km → GR {:.1} km, capture {:.1} km)",
            gr_shift / 1e3,
            baseline.perigee / 1e3,
            gr.perigee / 1e3,
            gr.capture_radius / 1e3,
        );
        assert!(
            gr_shift > 2.0e3,
            "1PN should move the perigee by a resolvable amount (> 2 km), got {:.3e} m",
            gr_shift
        );
        assert!(
            gr.perigee < gr.capture_radius,
            "GR-on perigee {:.1} km should still be a hit (inside capture {:.1} km)",
            gr.perigee / 1e3,
            gr.capture_radius / 1e3,
        );

        // (c) Yarkovsky at a physical, un-amplified A2 shifts the perigee by some
        //     nonzero finite amount — honest whether that is km-scale or sub-km.
        let yar = sc
            .nominal_encounter_with(&Tier2Config {
                relativity: false,
                yarkovsky_a2: Some(1.0e-13),
                ..Tier2Config::default()
            })
            .expect("Yarkovsky re-fly")
            .expect("Yarkovsky pass is still an encounter");
        let yar_shift = (yar.perigee - baseline.perigee).abs();
        println!(
            "Yarkovsky (A2 = 1e-13 m/s²) perigee shift over the campaign: {:.3} km",
            yar_shift / 1e3,
        );
        assert!(
            yar_shift > 0.0 && yar_shift.is_finite(),
            "a physical Yarkovsky A2 should move the perigee by a nonzero finite amount, got {:.3e} m",
            yar_shift
        );

        // (d) SRP at a physical, un-amplified area-to-mass shifts the perigee by
        //     some nonzero finite amount. SRP is radial (no secular along-track
        //     drift), so this is expected small — reported, not asserted large.
        let srp = sc
            .nominal_encounter_with(&Tier2Config {
                relativity: false,
                yarkovsky_a2: None,
                srp: Some(SrpParams::sub_km_rock()),
                ..Tier2Config::default()
            })
            .expect("SRP re-fly")
            .expect("SRP pass is still an encounter");
        let srp_shift = (srp.perigee - baseline.perigee).abs();
        println!(
            "SRP (sub-km rock, β≈2.5e-9) perigee shift over the campaign: {:.4} km",
            srp_shift / 1e3,
        );
        assert!(
            srp_shift > 0.0 && srp_shift.is_finite(),
            "a physical SRP term should move the perigee by a nonzero finite amount, got {:.3e} m",
            srp_shift
        );
    }

    /// The 16 sb441 asteroid perturbers, wired the same way GR and Yarkovsky are:
    /// enrolling them leaves the b-plane **unchanged when off** (the shipping demo
    /// invariant) and **shifts it by a measured amount when on**.
    ///
    /// Measured GR-style, on a **fixed Tier-1 seed**: the scenario is built all-off
    /// (so its seed is the shipping Tier-1 impactor) but on an ephemeris that *has*
    /// the sb441 kernel mounted, so `nominal_encounter_with` can re-compose the
    /// field with the asteroids added. Re-flying that one seed with the perturbers
    /// on is the direct measurement of how much the belt moves the predicted impact
    /// — reported, never asserted to a hand-derived magnitude (the shift is small;
    /// the belt is the residual *floor*, not a headline term).
    ///
    /// Kernel-gated **and** sb441-gated: skips (passes) if the DE pair or the
    /// optional small-body kernel is absent.
    #[test]
    fn asteroid_perturbers_leave_the_bplane_unchanged_off_and_shift_it_on() {
        let Some(k) = crate::kernels::resolve_for_test("asteroid_perturbers_…_shift_it_on") else {
            return;
        };
        let Some(sb) = k.small_bodies.clone() else {
            return; // sb441 is the optional 646 MB kernel; nothing to measure without it.
        };
        let (bsp, pca) = k.as_strs();
        // A Tier-1 seed (default config, asteroids off) but on an sb441-mounted
        // almanac, so the measurement path can add the perturbers to the same seed.
        let eph = Arc::new(
            Ephemeris::load(bsp)
                .expect("load DE kernel")
                .with_constants(pca)
                .expect("load constants")
                .with_constants(&sb)
                .expect("mount sb441"),
        );
        let sc = RealFieldScenario::build_with(&ImpactorConfig::default(), eph)
            .expect("Tier-1 scenario builds on an sb441-mounted almanac");

        // The shipping Tier-1 baseline: the nominal hit the built (all-off) scenario
        // reports, on an almanac that merely *has* sb441 available.
        let baseline = sc
            .deflection()
            .expect("deflection")
            .nominal_encounter()
            .expect("nominal reduces")
            .expect("nominal is a hit");

        // (a) Off re-fly == that baseline, to the last bit — the belt is not silently
        //     already in the field just because the kernel is mounted.
        let off = sc
            .nominal_encounter_with(&Tier2Config::default())
            .expect("off re-fly")
            .expect("off pass is still an encounter");
        assert_eq!(
            off.perigee, baseline.perigee,
            "asteroids-off re-fly must match the shipping Tier-1 perigee bit-for-bit"
        );

        // (b) Asteroids on shifts the perigee by a nonzero finite, measured amount.
        let ast = sc
            .nominal_encounter_with(&Tier2Config {
                asteroid_perturbers: true,
                ..Tier2Config::default()
            })
            .expect("asteroid re-fly")
            .expect("asteroid pass is still an encounter");
        let shift = (ast.perigee - baseline.perigee).abs();
        println!(
            "16 sb441 asteroid perturbers: perigee shift over the campaign {:.3} km \
             (baseline {:.1} km → +belt {:.1} km, capture {:.1} km)",
            shift / 1e3,
            baseline.perigee / 1e3,
            ast.perigee / 1e3,
            ast.capture_radius / 1e3,
        );
        assert!(
            shift > 0.0 && shift.is_finite(),
            "the belt should move the perigee by a nonzero finite amount, got {shift:.3e} m"
        );
    }
}
