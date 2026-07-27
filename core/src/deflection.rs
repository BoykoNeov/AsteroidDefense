//! `deflection` — the mission-planner primitive and the headline Δv solver
//! (HANDOFF §4, §5, §8, §10 task 10).
//!
//! This module is where the encounter pipeline becomes a *mission*. The core
//! does not plan trajectories; it exposes a single honest operation (§4): push a
//! Δv into the asteroid's state at a chosen time, re-propagate, and ask "did the
//! hit become a miss?". [`apply_impulse`] is that push; [`DeflectionScenario`]
//! is the run→mutate→re-run→compare loop around it.
//!
//! # The headline curve is a root-find, not a formula
//! The thesis screen (§1) is **required Δv vs. lead time**: for a deflection
//! applied at a given epoch, how small a nudge still turns the impact into an
//! acceptable miss? There is no closed form through the perturbed n-body field,
//! so [`DeflectionScenario::required_dv`] *solves* it — it grows the impulse
//! until the gravitationally-focused b-plane **perigee**
//! ([`BPlaneEncounter::perigee`]) clears a safe target, then bisects on the
//! impulse magnitude. Perigee — not raw geocentric closest-approach distance — is
//! the honest hit/miss metric, because Earth's gravity focuses the capture cross
//! section (§10.8); solving on the raw miss would let a "miss" that Earth still
//! reels in count as safe.
//!
//! # Fixed phase, along-track direction (MVP)
//! A kinetic impactor mostly imparts an **along-track** Δv (§5), and the headline
//! curve fixes the impulse *phase* and *direction* so the one varying quantity is
//! lead time (§7, resolved 2026-06-23). [`along_track_unit`] gives that
//! direction from the state at the deflection epoch;
//! [`DeflectionScenario::required_dv`] also accepts an explicit direction for the
//! separate phase-sensitivity view. Non-monotone perigee-vs-Δv from keyhole
//! geometry is a Tier-3 phenomenon (§ open questions) and out of scope here: the
//! solver assumes the perigee grows with the impulse over the bracket it finds.
//!
//! # Kernel-free by construction
//! Everything here composes [`Clock`], [`closest_approach`], and
//! [`BPlaneEncounter`] over a caller-supplied [`ForceModel`] and
//! [`GeocentricState`]. The shipping viewer wires the ANISE DE440 field and the
//! reconstructed geocentre; the tests wire a straight-line drift and a Sun-only
//! two-body orbit against a synthetic Earth — the same doubles the
//! close-approach and geometry tests use — so the mission machinery is validated
//! with no ephemeris kernel present.

use nalgebra::Vector3;

use crate::clock::{Clock, ClockError};
use crate::close_approach::{closest_approach, GeocentricState, ScanOptions};
use crate::epoch::Epoch;
use crate::forces::{ForceModel, GRAVITATIONAL_CONSTANT};
use crate::geometry::{BPlaneEncounter, GeometryError};
use crate::integrator::{Dop853, IntegratorError};
use crate::state::StateVector;

/// Apply an instantaneous Δv (m/s, SSB ICRF) to `state`, adding it to the
/// velocity and leaving the position unchanged.
///
/// This is the mission planner's entire coupling to the physics (§4): a
/// deflection — whatever its mechanism — reduces to an impulse the core folds
/// into the state before re-propagating. Instantaneous is the MVP idealization;
/// a finite burn (gravity tractor, §5) is a Tier-2 force term, not this.
pub fn apply_impulse(state: StateVector, delta_v: Vector3<f64>) -> StateVector {
    StateVector::new(state.position, state.velocity + delta_v)
}

/// The unit along-track (prograde) direction `v̂` at `state` — the direction the
/// MVP kinetic impactor's Δv is modeled along for the headline curve (§5, §7).
///
/// Returns `None` for a state with zero velocity, which has no defined heading.
pub fn along_track_unit(state: StateVector) -> Option<Vector3<f64>> {
    let speed = state.velocity.norm();
    if speed > 0.0 {
        Some(state.velocity / speed)
    } else {
        None
    }
}

/// The along-track Δv magnitude (m/s) a kinetic impactor imparts under the
/// momentum-transfer model (§5, §8):
///
/// ```text
///   |Δv| = β · (m_impactor / M_asteroid) · v_rel
/// ```
///
/// `beta` (β ≥ 1) is the momentum-enhancement factor from ejecta — β = 1 is pure
/// momentum transfer, and DART measured β ≈ 3.6 at Dimorphos (§9). This produces
/// the Δv *magnitude* the solver consumes; it is deliberately not a force term.
/// [`StandoffNuclear`] is the high-Δv end of the same spectrum (§5); the
/// gravity-tractor end is a force term, not an impulse, and is not here.
pub fn kinetic_impactor_dv(
    beta: f64,
    impactor_mass_kg: f64,
    relative_speed_ms: f64,
    asteroid_mass_kg: f64,
) -> f64 {
    beta * (impactor_mass_kg / asteroid_mass_kg) * relative_speed_ms
}

/// The impactor mass (kg) that imparts `dv_ms` — the exact inverse of
/// [`kinetic_impactor_dv`], so a required Δv can be quoted in kilograms.
///
/// This is a *closed-form* invert and deliberately not a root-find: the momentum
/// model is linear in mass. [`crate::mission::required_impactor_mass`] solves
/// rather than inverts because it works in the **coupled** direction, where the
/// arrival `v_rel` is fixed by the transfer and only its along-track projection
/// deflects — there the perigee-vs-mass curve runs through the full n-body field
/// and has no inverse. Along a chosen direction, this is arithmetic.
///
/// Returns `None` for a non-positive or non-finite denominator (β, `v_rel`, or M).
pub fn kinetic_impactor_mass_for_dv(
    dv_ms: f64,
    beta: f64,
    relative_speed_ms: f64,
    asteroid_mass_kg: f64,
) -> Option<f64> {
    let denom = beta * relative_speed_ms;
    let ok = denom.is_finite()
        && denom > 0.0
        && asteroid_mass_kg.is_finite()
        && asteroid_mass_kg > 0.0
        && dv_ms.is_finite();
    if !ok {
        return None;
    }
    Some(dv_ms * asteroid_mass_kg / denom)
}

// --- Nuclear standoff burst (§5) ---------------------------------------------

/// Joules per kilotonne of TNT equivalent. Definitional (1 kt ≡ 10⁹ cal at
/// 4.184 J/cal), not measured — exact by convention.
pub const JOULES_PER_KILOTONNE: f64 = 4.184e12;

/// A standoff nuclear burst, modeled as **energy deposited → surface ablation →
/// momentum → Δv** (§5) — deflection physics only, never device design.
///
/// # The model
/// A device of yield `Y` detonated at a standoff height above one hemisphere
/// radiates neutrons and X-rays into a thin surface layer. A fraction
/// [`coupling_efficiency`](Self::coupling_efficiency) of the yield is absorbed;
/// the ablated layer leaves at kilometres per second, and the recoil is
///
/// ```text
///   Δv = C_m · η · Y / M
/// ```
///
/// with `C_m` the momentum delivered per joule *absorbed*
/// ([`momentum_coupling_ns_per_j`](Self::momentum_coupling_ns_per_j)) and `M` the
/// asteroid mass. Both coefficients are pinned to published simulation output —
/// see [`DEARBORN_2007`](Self::DEARBORN_2007).
///
/// # Why yield is the axis, and mass is not
/// The knob here is **yield**, taken as an opaque scalar. Yield does not scale
/// linearly with device mass, and the mass→yield map is precisely where §5's
/// *"model this as deflection physics only — never weapon design"* bites. So a
/// launch window's delivered mass never converts to a yield in this crate; it can
/// only gate whether a device of some stated class is carriable at all, which is a
/// payload question the mission layer already answers.
///
/// # Direction is chosen, and that is the physical difference from a kinetic impactor
/// A kinetic impactor's Δv points along the arrival relative velocity — the
/// geometry of the transfer picks the direction, and the mission layer can only
/// take the along-track *projection* it happens to get
/// ([`crate::mission::impact_impulse`]). A standoff burst ablates whichever
/// hemisphere the device is placed over, so its impulse direction is a mission
/// choice. That is why this term is modeled as a Δv **magnitude** applied along a
/// direction the caller picks, and why the headline comparison quotes it
/// along-track: not an idealization glossing over geometry, but the geometry
/// actually being free.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StandoffNuclear {
    /// Fraction of device yield absorbed by the surface (0 < η ≤ 1). Folds the
    /// solid angle the body subtends at the burst height together with the
    /// material's absorption of the neutron/X-ray flux.
    pub coupling_efficiency: f64,
    /// Momentum imparted per joule **absorbed**, N·s/J. The ablation physics:
    /// how much of the deposited energy leaves as directed ejecta momentum
    /// rather than radiation or internal heat.
    pub momentum_coupling_ns_per_j: f64,
}

impl StandoffNuclear {
    /// Coefficients derived from the **"Nudge Model"** of D. S. P. Dearborn,
    /// *The Use of Nuclear Explosives To Disrupt or Divert Asteroids*,
    /// UCRL-PROC-228569, Lawrence Livermore National Laboratory, 2007
    /// (<https://www.osti.gov/servlets/purl/902607>).
    ///
    /// # The published case, in full
    /// - Body: the paper's "standard structure" — **1000 m diameter**, 250 m
    ///   granite core (2.63 g/cc) inside a 250 m tuff mantle (1.91 g/cc), total
    ///   mass **1.05 × 10¹² kg**, surface escape speed ≈ **0.5 m/s**.
    /// - Device: **100 kt** at **300 m** above the surface. That height is
    ///   `d/R = 0.6`, which the same paper independently names as the **optimum
    ///   height of burst**, so this fixture sits at the geometry the model is
    ///   meant to describe rather than at an arbitrary one.
    /// - Energy sourced into the surface zones: **11.5 kt** → η = 11.5/100 =
    ///   **0.115**.
    /// - Outcome: net speed change of the coalesced body **≈ 6.5 mm/s**, with
    ///   **99 %** of the mass remaining bound.
    ///
    /// Inverting that single case for the momentum coupling gives
    /// `C_m = M·Δv / E_abs = 1.05e12 × 6.5e-3 / (11.5 × 4.184e12)` =
    /// **1.418 × 10⁻⁴ N·s/J**.
    ///
    /// # It is cross-checked against a different paper's different simulation
    /// Dearborn & Bruck, *Limits on the use of nuclear explosives for asteroid
    /// deflection*, LLNL-PROC-485160 (Planetary Defense Conference, 2011), report
    /// for a separate run that *"[t]en kilotons deposition then converts about
    /// 4000 tons of surface material into plasma expanding at over 2 km/s"* —
    /// which is `C_m = 4.0e6 × 2000 / (10 × 4.184e12)` = **1.91 × 10⁻⁴ N·s/J**.
    ///
    /// The two disagree by **at least 36 %**, and "at least" is the operative
    /// word: *"over* 2 km/s" makes the 2011 value a **lower bound**, `C_m ≥
    /// 1.91e-4`, and the shipped 1.418e-4 sits *below* that bound. So the two
    /// figures do **not** bracket the truth between them — they point the same
    /// way, and 36 % is the floor of the disagreement, not a range containing it.
    /// **That is the honest uncertainty of this term**: a coefficient good to
    /// something like a factor of two, not to three digits, and nothing
    /// downstream should quote it as though it were better.
    ///
    /// `1.418e-4` is used because it comes from the fully-specified case (body,
    /// yield, height, absorbed energy and outcome all stated together) rather
    /// than from one parenthetical clause — and being the smaller value it
    /// demands *more* yield for a given Δv, so the requirement errs high.
    pub const DEARBORN_2007: Self = Self {
        coupling_efficiency: 0.115,
        momentum_coupling_ns_per_j: 1.418e-4,
    };

    /// The along-burst Δv magnitude (m/s) a device of `yield_kilotonnes` imparts
    /// to a body of `asteroid_mass_kg`.
    ///
    /// Returns `None` on non-finite or non-positive inputs. **Says nothing about
    /// whether the body survives** — pair it with [`disruption_regime`], because
    /// the published model is a small-nudge model and stops being one well before
    /// the yield stops rising.
    pub fn dv_ms(&self, yield_kilotonnes: f64, asteroid_mass_kg: f64) -> Option<f64> {
        let ok = self.is_valid()
            && yield_kilotonnes.is_finite()
            && yield_kilotonnes >= 0.0
            && asteroid_mass_kg.is_finite()
            && asteroid_mass_kg > 0.0;
        if !ok {
            return None;
        }
        let absorbed_j = self.coupling_efficiency * yield_kilotonnes * JOULES_PER_KILOTONNE;
        Some(self.momentum_coupling_ns_per_j * absorbed_j / asteroid_mass_kg)
    }

    /// The yield (kilotonnes) needed for `dv_ms` — the closed-form inverse of
    /// [`dv_ms`](Self::dv_ms), which is what lets the required-Δv curve be quoted
    /// in kilotonnes without a second root-find.
    pub fn yield_kilotonnes_for_dv(&self, dv_ms: f64, asteroid_mass_kg: f64) -> Option<f64> {
        let ok = self.is_valid()
            && dv_ms.is_finite()
            && dv_ms >= 0.0
            && asteroid_mass_kg.is_finite()
            && asteroid_mass_kg > 0.0;
        if !ok {
            return None;
        }
        let per_kt =
            self.momentum_coupling_ns_per_j * self.coupling_efficiency * JOULES_PER_KILOTONNE
                / asteroid_mass_kg;
        if !(per_kt.is_finite() && per_kt > 0.0) {
            return None;
        }
        Some(dv_ms / per_kt)
    }

    fn is_valid(&self) -> bool {
        self.coupling_efficiency.is_finite()
            && self.coupling_efficiency > 0.0
            && self.coupling_efficiency <= 1.0
            && self.momentum_coupling_ns_per_j.is_finite()
            && self.momentum_coupling_ns_per_j > 0.0
    }
}

/// Surface escape speed (m/s) of a uniform sphere — `sqrt(2GM/R)`.
///
/// The bar the nuclear term is judged against: an impulse that is a large
/// fraction of this does not deflect a body, it takes it apart.
pub fn escape_speed_ms(mass_kg: f64, radius_m: f64) -> Option<f64> {
    let ok = mass_kg.is_finite() && mass_kg > 0.0 && radius_m.is_finite() && radius_m > 0.0;
    if !ok {
        return None;
    }
    Some((2.0 * GRAVITATIONAL_CONSTANT * mass_kg / radius_m).sqrt())
}

/// What a given Δv does to the body, in the terms the published simulations
/// actually report.
///
/// The sources are emphatic that a standoff nudge is only a *nudge* while Δv stays
/// far below the escape speed, and they give two anchor points — no curve between
/// them. This enum refuses to invent one: see [`disruption_regime`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisruptionRegime {
    /// At or below the ratio a published run demonstrated leaves the body intact.
    IntactDeflection,
    /// Between the two published anchors — **the sources do not say**. Not a
    /// prediction of marginality; an admission that this range is uncharacterised.
    Uncharacterised,
    /// At or above the ratio a published run demonstrated fragments the body.
    LikelyDisruption,
}

/// Ratio of Δv to escape speed at or below which a published run kept the body
/// intact: the UCRL-PROC-228569 "Nudge Model" — 6.5 mm/s against a ≈ 0.5 m/s
/// escape speed, **99 % of the mass remaining bound**.
///
/// Quoted from the paper's own *rounded* 0.5 m/s. Recomputing the escape speed
/// from the same paper's stated mass and diameter gives 0.5295 m/s and hence a
/// ratio of 0.01228 — so the Nudge Model case clears this threshold by only
/// **≈6 %**. That is thin for a bound defined from that very case, and it is
/// stated rather than tuned away: widening the constant to fit its own fixture
/// more comfortably would be choosing a number the sources did not choose, which
/// is the thing the `Uncharacterised` band exists to avoid.
pub const INTACT_DV_OVER_VESC: f64 = 0.013;

/// Ratio of Δv to escape speed at or above which a published run reported
/// fragmentation: LLNL-PROC-485160 — *"[a]t a size of 100 meters, the escape
/// speed is near 5 cm/s, and inducing a 1 cm/s speed change will almost certainly
/// result in extensive debris ejection or fragmentation."*
pub const DISRUPTION_DV_OVER_VESC: f64 = 0.2;

/// Classify an impulse against the body's own gravity.
///
/// # Why there is an `Uncharacterised` band instead of a threshold
/// A single cut would have to be *chosen*, and nothing in the sources chooses it.
/// What they supply is two demonstrated points fifteen-fold apart — intact at
/// `Δv/v_esc = 0.013`, fragmented at `0.2` — so the honest classifier reports
/// which side of *those* a case falls on and names the gap as a gap. A tidy
/// midpoint would read as knowledge and would be invention.
///
/// Returns `None` if the escape speed is undefined (non-positive mass or radius).
pub fn disruption_regime(dv_ms: f64, mass_kg: f64, radius_m: f64) -> Option<DisruptionRegime> {
    let v_esc = escape_speed_ms(mass_kg, radius_m)?;
    if !(dv_ms.is_finite() && dv_ms >= 0.0) {
        return None;
    }
    let ratio = dv_ms / v_esc;
    Some(if ratio <= INTACT_DV_OVER_VESC {
        DisruptionRegime::IntactDeflection
    } else if ratio >= DISRUPTION_DV_OVER_VESC {
        DisruptionRegime::LikelyDisruption
    } else {
        DisruptionRegime::Uncharacterised
    })
}

/// Tuning for the [`DeflectionScenario::required_dv`] root-find.
///
/// [`Default`] seeds the bracket at 1 mm/s, doubles it to find the crossing, and
/// stops when the impulse bracket is pinned to 0.1 % — ample for a legible curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DvSolveTol {
    /// First upper-bracket probe magnitude, m/s (> 0). The search doubles up from
    /// here, so the exact value only trades a few evaluations, not correctness.
    pub v_seed: f64,
    /// Geometric growth factor for the bracket expansion (> 1).
    pub growth: f64,
    /// Relative half-width at which the bisection stops:
    /// `(v_hi − v_lo) ≤ rel_tol · v_hi`.
    pub rel_tol: f64,
    /// Cap on doublings while hunting the upper bracket — a runaway backstop for a
    /// target the impulse can never reach (e.g. beyond the propagated span).
    pub max_expansions: u32,
    /// Cap on bisection halvings (the `rel_tol` test normally stops it first).
    pub max_bisections: u32,
}

impl Default for DvSolveTol {
    fn default() -> Self {
        Self {
            v_seed: 1.0e-3,
            growth: 2.0,
            rel_tol: 1.0e-3,
            max_expansions: 64,
            max_bisections: 100,
        }
    }
}

/// Why a deflection evaluation or solve could not complete.
#[derive(Debug, Clone, PartialEq)]
pub enum DeflectionError {
    /// Sampling the nominal trajectory at the deflection epoch failed (epoch
    /// outside the propagated span).
    Clock(ClockError),
    /// Re-propagating the deflected trajectory failed.
    Integrator(IntegratorError),
    /// The close-approach scan over the deflected trajectory failed.
    CloseApproach(crate::close_approach::CloseApproachError),
    /// Reducing the located encounter to b-plane geometry failed (e.g. a
    /// gravitationally-bound relative pass with no hyperbolic `v_inf`).
    Geometry(GeometryError),
    /// The requested impulse direction was the zero vector, or the along-track
    /// convenience was asked for a state with no velocity.
    NoDirection,
    /// An input was out of range (non-finite / non-positive tolerance, target,
    /// or scenario parameter). Carries a human-readable reason.
    InvalidInput(String),
    /// The bracket expansion hit [`DvSolveTol::max_expansions`] without the
    /// perigee ever clearing the target — the target miss is unreachable within
    /// the propagated span (extend the span or relax the target).
    Unreachable {
        /// Target perigee that was never cleared, metres.
        target_perigee_m: f64,
        /// Largest impulse magnitude probed, m/s.
        max_dv_tried: f64,
        /// Perigee reached at that impulse, metres (may be `INFINITY` if the
        /// deflected pass left the scan's distance gate).
        perigee_reached_m: f64,
    },
}

impl std::fmt::Display for DeflectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DeflectionError::Clock(e) => write!(f, "sampling nominal trajectory failed: {e}"),
            DeflectionError::Integrator(e) => write!(f, "re-propagation failed: {e}"),
            DeflectionError::CloseApproach(e) => write!(f, "close-approach scan failed: {e}"),
            DeflectionError::Geometry(e) => write!(f, "b-plane reduction failed: {e}"),
            DeflectionError::NoDirection => write!(f, "impulse direction is undefined (zero vector)"),
            DeflectionError::InvalidInput(m) => write!(f, "invalid input: {m}"),
            DeflectionError::Unreachable {
                target_perigee_m,
                max_dv_tried,
                perigee_reached_m,
            } => write!(
                f,
                "target perigee {target_perigee_m:.3e} m unreachable: at |Δv| = {max_dv_tried:.3e} m/s \
                 perigee only reached {perigee_reached_m:.3e} m"
            ),
        }
    }
}

impl std::error::Error for DeflectionError {}

impl From<ClockError> for DeflectionError {
    fn from(e: ClockError) -> Self {
        DeflectionError::Clock(e)
    }
}
impl From<IntegratorError> for DeflectionError {
    fn from(e: IntegratorError) -> Self {
        DeflectionError::Integrator(e)
    }
}
impl From<crate::close_approach::CloseApproachError> for DeflectionError {
    fn from(e: crate::close_approach::CloseApproachError) -> Self {
        DeflectionError::CloseApproach(e)
    }
}
impl From<GeometryError> for DeflectionError {
    fn from(e: GeometryError) -> Self {
        DeflectionError::Geometry(e)
    }
}

/// A deflection scenario: a nominal (un-deflected) asteroid trajectory plus the
/// machinery to re-propagate it after an impulse and read off the resulting
/// Earth encounter.
///
/// Construction propagates the nominal trajectory **once**
/// ([`DeflectionScenario::new`]); every evaluation then samples the nominal
/// state at the deflection epoch, applies the impulse, and integrates a *fresh*
/// clock forward from there to the encounter. Sweeping the headline curve over
/// many lead times therefore re-integrates only the post-deflection arc each
/// time, not the cruise up to it.
///
/// The force model, Earth-state source, integrator tolerances, snapshot cadence,
/// scan options, and Earth `μ`/`R` are all caller-supplied, so the same type
/// drives the kernel-free tests and the ANISE DE440 shipping build unchanged.
pub struct DeflectionScenario<'a> {
    integrator: Dop853,
    force: &'a dyn ForceModel,
    earth: &'a dyn GeocentricState,
    cadence_seconds: f64,
    /// End of the propagated span, seconds past J2000 (TDB). A deflected
    /// re-propagation runs from the deflection epoch to here.
    span_end_seconds: f64,
    scan: ScanOptions,
    mu_earth: f64,
    earth_radius: f64,
    /// The nominal trajectory, propagated once from the seed over the full span.
    nominal: Clock,
}

impl<'a> DeflectionScenario<'a> {
    /// Build a scenario by propagating the nominal trajectory from `state0` at
    /// `epoch0` under `force`, taking `n_snapshots` steps of `cadence_seconds`.
    ///
    /// `cadence_seconds` must be positive (the scenario propagates forward, from
    /// the deflection epoch toward the encounter); `n_snapshots ≥ 1`; the span
    /// `n_snapshots · cadence_seconds` must extend a margin **past** the nominal
    /// encounter so a deflected (time-shifted) pass still lands inside it.
    /// `mu_earth` (m³/s²) and `earth_radius` (m) drive the b-plane reduction.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        integrator: Dop853,
        force: &'a dyn ForceModel,
        earth: &'a dyn GeocentricState,
        epoch0: Epoch,
        state0: StateVector,
        cadence_seconds: f64,
        n_snapshots: u32,
        scan: ScanOptions,
        mu_earth: f64,
        earth_radius: f64,
    ) -> Result<Self, DeflectionError> {
        // Validate before propagating: `Clock::propagate` *asserts* on a bad
        // cadence, so the checks must reject it as an error first.
        Self::validate(cadence_seconds, n_snapshots, mu_earth, earth_radius)?;
        let nominal = Clock::propagate(
            &integrator,
            force,
            epoch0,
            state0,
            cadence_seconds,
            n_snapshots,
        )?;
        Self::with_nominal(
            integrator,
            force,
            earth,
            epoch0,
            nominal,
            cadence_seconds,
            n_snapshots,
            scan,
            mu_earth,
            earth_radius,
        )
    }

    /// Build from an **already-propagated** nominal trajectory, skipping the
    /// multi-year propagation [`new`](Self::new) performs.
    ///
    /// The nominal is a pure function of the seed and the force field, and it
    /// never changes — but `new` re-flies it on every construction, so a caller
    /// that builds a scenario per interaction (a planner re-evaluating on each
    /// nudge) pays the whole cruise each time to learn something it already knows.
    /// [`RealFieldScenario::deflection`](crate::scenario::RealFieldScenario::deflection)
    /// propagates once, caches, and comes through here.
    ///
    /// **Contract:** `nominal` must be the trajectory propagated from *this*
    /// scenario's seed under *this* `force`, at this `epoch0`/cadence — nothing
    /// checks that, and a foreign clock would silently report the wrong
    /// encounter. `pub(crate)` for exactly that reason: the only caller is the
    /// scenario that owns both the seed and the field, so the pairing cannot be
    /// wrong. `new` remains the safe public constructor.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn with_nominal(
        integrator: Dop853,
        force: &'a dyn ForceModel,
        earth: &'a dyn GeocentricState,
        epoch0: Epoch,
        nominal: Clock,
        cadence_seconds: f64,
        n_snapshots: u32,
        scan: ScanOptions,
        mu_earth: f64,
        earth_radius: f64,
    ) -> Result<Self, DeflectionError> {
        Self::validate(cadence_seconds, n_snapshots, mu_earth, earth_radius)?;
        let span_end_seconds = epoch0
            .shifted_by_seconds(cadence_seconds * n_snapshots as f64)
            .tdb_seconds_past_j2000();

        Ok(Self {
            integrator,
            force,
            earth,
            cadence_seconds,
            span_end_seconds,
            scan,
            mu_earth,
            earth_radius,
            nominal,
        })
    }

    /// The construction preconditions shared by [`new`](Self::new) and
    /// [`with_nominal`](Self::with_nominal), so the two constructors cannot drift
    /// on what they accept.
    pub(crate) fn validate(
        cadence_seconds: f64,
        n_snapshots: u32,
        mu_earth: f64,
        earth_radius: f64,
    ) -> Result<(), DeflectionError> {
        if !(cadence_seconds.is_finite() && cadence_seconds > 0.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "cadence_seconds must be finite and > 0 (got {cadence_seconds})"
            )));
        }
        if n_snapshots < 1 {
            return Err(DeflectionError::InvalidInput(
                "n_snapshots must be at least 1".into(),
            ));
        }
        if !(mu_earth.is_finite() && mu_earth > 0.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "mu_earth must be finite and > 0 (got {mu_earth})"
            )));
        }
        if !(earth_radius.is_finite() && earth_radius > 0.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "earth_radius must be finite and > 0 (got {earth_radius})"
            )));
        }
        Ok(())
    }

    /// The nominal trajectory (read-only) — handy for the animation and for
    /// sampling the un-deflected state at any covered epoch.
    pub fn nominal(&self) -> &Clock {
        &self.nominal
    }

    /// The nominal (un-deflected) Earth encounter as b-plane geometry, or `None`
    /// if no close approach falls inside the scan's distance gate.
    ///
    /// For an impact scenario this is the hit the mission is trying to undo.
    pub fn nominal_encounter(&self) -> Result<Option<BPlaneEncounter>, DeflectionError> {
        let ca = closest_approach(&self.nominal, self.earth, self.scan)?;
        match ca {
            Some(c) => Ok(Some(c.b_plane(self.mu_earth, self.earth_radius)?)),
            None => Ok(None),
        }
    }

    /// Apply `delta_v` (m/s, SSB ICRF) at `deflection_epoch`, re-propagate from
    /// there, and return the resulting Earth encounter as b-plane geometry — or
    /// `None` if the deflected pass left the scan's distance gate (a clean miss).
    ///
    /// `deflection_epoch` must lie within the nominal span (the nominal state
    /// there is the seed the impulse is added to).
    pub fn evaluate(
        &self,
        deflection_epoch: Epoch,
        delta_v: Vector3<f64>,
    ) -> Result<Option<BPlaneEncounter>, DeflectionError> {
        let (_clock, encounter) = self.deflected_trajectory(deflection_epoch, delta_v)?;
        Ok(encounter)
    }

    /// Like [`evaluate`](Self::evaluate), but also hands back the propagated
    /// post-deflection [`Clock`] — the sampled deflected arc itself, not just the
    /// encounter it produces.
    ///
    /// The animation needs the trajectory, not only its perigee: it draws the
    /// deflected path against the nominal one and against Earth to *show* the
    /// hit becoming a miss. [`evaluate`](Self::evaluate) is defined in terms of
    /// this method (discarding the clock), so the two can never disagree on the
    /// encounter — the displayed track and the reported b-plane perigee come from
    /// one propagation.
    pub fn deflected_trajectory(
        &self,
        deflection_epoch: Epoch,
        delta_v: Vector3<f64>,
    ) -> Result<(Clock, Option<BPlaneEncounter>), DeflectionError> {
        let t_d = deflection_epoch.tdb_seconds_past_j2000();
        if t_d >= self.span_end_seconds {
            return Err(DeflectionError::InvalidInput(format!(
                "deflection epoch ({t_d} s) is at or past the span end ({} s)",
                self.span_end_seconds
            )));
        }

        let seed = self.nominal.state_at(deflection_epoch)?;
        let deflected = apply_impulse(seed, delta_v);

        // Re-propagate from the deflection epoch to the span end, at the same
        // cadence. Round the interval count up so the clock always reaches
        // `span_end` (and thus past the nominal encounter); at least one step.
        let remaining = ((self.span_end_seconds - t_d) / self.cadence_seconds).ceil();
        let n = (remaining as u32).max(1);
        let clock = Clock::propagate(
            &self.integrator,
            self.force,
            deflection_epoch,
            deflected,
            self.cadence_seconds,
            n,
        )?;

        let ca = closest_approach(&clock, self.earth, self.scan)?;
        let encounter = match ca {
            Some(c) => Some(c.b_plane(self.mu_earth, self.earth_radius)?),
            None => None,
        };
        Ok((clock, encounter))
    }

    /// The b-plane perigee (m) after a `magnitude`·`direction` impulse at
    /// `deflection_epoch` — the monotone-ish quantity the
    /// [`required_dv`](Self::required_dv) root-find brackets on. Two encounter
    /// outcomes are folded into the perigee scale at its extremes:
    ///
    /// - A pass that left the scan's distance gate ⇒ `+∞` (it missed by more than
    ///   we scan for — cleanly above any target).
    /// - A pass so deep that the osculating relative orbit is *not* hyperbolic
    ///   ([`GeometryError::NotHyperbolic`]) ⇒ `0.0`. For a fast NEO encounter a
    ///   bound relative orbit means the sampled closest approach fell inside
    ///   `2μ⊕/v²` (a few hundred km of Earth's centre) — a dead-centre hit, the
    ///   worst possible perigee. Sweeping the impulse can pass *through* such a
    ///   near-collision on its way to opening a miss, so the solver must read it
    ///   as "still a hit," not fail. (A genuinely slow, gravitationally-bound
    ///   co-orbital pass would also land here; that is a Tier-3 regime, not the
    ///   direct-impact geometry this solver models.)
    fn perigee_after(
        &self,
        deflection_epoch: Epoch,
        direction: Vector3<f64>,
        magnitude: f64,
    ) -> Result<f64, DeflectionError> {
        match self.evaluate(deflection_epoch, magnitude * direction) {
            Ok(Some(bp)) => Ok(bp.perigee),
            Ok(None) => Ok(f64::INFINITY),
            Err(DeflectionError::Geometry(GeometryError::NotHyperbolic { .. })) => Ok(0.0),
            Err(e) => Err(e),
        }
    }

    /// Solve for the impulse **magnitude** (m/s) along `direction` that raises the
    /// gravitationally-focused b-plane perigee to `target_perigee_m`, when
    /// applied at `deflection_epoch`.
    ///
    /// Grows the impulse geometrically from [`DvSolveTol::v_seed`] until the
    /// perigee clears the target, then bisects the magnitude to
    /// [`DvSolveTol::rel_tol`]. Returns `0.0` if the un-deflected perigee already
    /// clears the target (no impulse needed). `direction` need not be unit — it
    /// is normalized; the zero vector is [`DeflectionError::NoDirection`].
    ///
    /// The perigee is assumed to grow with the impulse over the bracket found
    /// (true for an along-track nudge on a direct-impact trajectory); keyhole
    /// non-monotonicity is Tier-3 and not modeled here.
    pub fn required_dv(
        &self,
        deflection_epoch: Epoch,
        direction: Vector3<f64>,
        target_perigee_m: f64,
        tol: DvSolveTol,
    ) -> Result<f64, DeflectionError> {
        if !(target_perigee_m.is_finite() && target_perigee_m > 0.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "target_perigee_m must be finite and > 0 (got {target_perigee_m})"
            )));
        }
        if !(tol.v_seed.is_finite() && tol.v_seed > 0.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "v_seed must be finite and > 0 (got {})",
                tol.v_seed
            )));
        }
        if !(tol.growth.is_finite() && tol.growth > 1.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "growth must be finite and > 1 (got {})",
                tol.growth
            )));
        }
        if !(tol.rel_tol.is_finite() && tol.rel_tol > 0.0) {
            return Err(DeflectionError::InvalidInput(format!(
                "rel_tol must be finite and > 0 (got {})",
                tol.rel_tol
            )));
        }

        let dir_norm = direction.norm();
        if !(dir_norm.is_finite() && dir_norm > 0.0) {
            return Err(DeflectionError::NoDirection);
        }
        let dir = direction / dir_norm;

        // At zero impulse: if the nominal already clears the target, no nudge is
        // required. (Also covers a nominal that was never a hit.)
        let p0 = self.perigee_after(deflection_epoch, dir, 0.0)?;
        if p0 >= target_perigee_m {
            return Ok(0.0);
        }

        // Geometric bracket expansion: grow the magnitude until the perigee
        // clears the target. `v_lo`/`v_hi` bracket the crossing (perigee below /
        // at-or-above the target).
        let mut v_hi = tol.v_seed;
        let mut p_hi = self.perigee_after(deflection_epoch, dir, v_hi)?;
        let mut expansions = 0u32;
        while p_hi < target_perigee_m {
            if expansions >= tol.max_expansions {
                return Err(DeflectionError::Unreachable {
                    target_perigee_m,
                    max_dv_tried: v_hi,
                    perigee_reached_m: p_hi,
                });
            }
            v_hi *= tol.growth;
            p_hi = self.perigee_after(deflection_epoch, dir, v_hi)?;
            expansions += 1;
        }

        // Bisect [v_lo, v_hi] for perigee == target. v_lo = 0 is known below
        // (p0 < target); v_hi is known at-or-above.
        let mut v_lo = 0.0_f64;
        for _ in 0..tol.max_bisections {
            if (v_hi - v_lo) <= tol.rel_tol * v_hi {
                break;
            }
            let v_mid = 0.5 * (v_lo + v_hi);
            if self.perigee_after(deflection_epoch, dir, v_mid)? < target_perigee_m {
                v_lo = v_mid;
            } else {
                v_hi = v_mid;
            }
        }
        Ok(0.5 * (v_lo + v_hi))
    }

    /// [`required_dv`](Self::required_dv) with the impulse fixed **along-track**
    /// (`v̂` at the deflection epoch) — the headline-curve direction (§5, §7).
    ///
    /// [`DeflectionError::NoDirection`] if the nominal state at the deflection
    /// epoch has no velocity (no defined heading).
    pub fn required_dv_along_track(
        &self,
        deflection_epoch: Epoch,
        target_perigee_m: f64,
        tol: DvSolveTol,
    ) -> Result<f64, DeflectionError> {
        let seed = self.nominal.state_at(deflection_epoch)?;
        let dir = along_track_unit(seed).ok_or(DeflectionError::NoDirection)?;
        self.required_dv(deflection_epoch, dir, target_perigee_m, tol)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::forces::{ForceError, ForceModel};
    use nalgebra::Vector3;

    /// Earth GM (m³/s²), DE440-consistent — the fixed literal the kernel-free
    /// `close_approach`/`geometry` tests use.
    const MU_EARTH: f64 = 3.986_004_356e14;
    /// Sun GM (m³/s²), DE440-consistent (Park et al. 2021).
    const MU_SUN: f64 = 1.327_124_400_18e20;
    /// Earth equatorial radius (m).
    const R_EARTH: f64 = crate::geometry::EARTH_EQUATORIAL_RADIUS_M;
    /// One astronomical unit (m), DE440.
    const AU_M: f64 = 1.495_978_707e11;

    /// A zero-acceleration field: the asteroid drifts in a straight line, so the
    /// mission machinery can be checked against exact geometry with no physics in
    /// the way. Shared with the `close_approach` tests in spirit.
    struct ZeroForce;
    impl ForceModel for ZeroForce {
        fn acceleration(
            &self,
            _epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(Vector3::zeros())
        }
    }

    /// A Sun-only two-body field: one attractor of `MU_SUN` pinned at the frame
    /// origin, so the asteroid follows a clean heliocentric Kepler orbit while the
    /// synthetic Earth is a massless target point (matching the codebase's
    /// test-particle-past-a-massless-Earth convention).
    fn sun_only() -> PointMassGravity {
        PointMassGravity::new(vec![(MU_SUN, FixedPerturber::at_origin()).into()])
    }

    // ---- Test 1: the impulse primitive and the kinetic β model ---------------

    #[test]
    fn apply_impulse_adds_to_velocity_only() {
        let s = StateVector::from_components(1.0, 2.0, 3.0, 10.0, 20.0, 30.0);
        let out = apply_impulse(s, Vector3::new(0.5, -1.0, 2.0));
        assert_eq!(out.position, s.position, "position must be untouched");
        assert_eq!(out.velocity, Vector3::new(10.5, 19.0, 32.0));
    }

    #[test]
    fn along_track_is_unit_prograde_and_none_at_rest() {
        let s = StateVector::from_components(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        let u = along_track_unit(s).expect("moving state has a heading");
        assert!((u.norm() - 1.0).abs() < 1e-15);
        assert!((u - Vector3::new(0.6, 0.8, 0.0)).norm() < 1e-15);

        let at_rest = StateVector::from_components(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
        assert!(along_track_unit(at_rest).is_none());
    }

    #[test]
    fn kinetic_impactor_dv_is_beta_mass_ratio_times_speed() {
        // β=3.6, 500 kg impactor at 6 km/s into a 1e9 kg asteroid.
        let dv = kinetic_impactor_dv(3.6, 500.0, 6_000.0, 1.0e9);
        let expected = 3.6 * (500.0 / 1.0e9) * 6_000.0;
        assert!((dv - expected).abs() < 1e-18);
        assert!(dv > 0.0);
    }

    // ---- Test 2: solver machinery on an exact straight-line pass -------------

    /// A straight-line drift toward a fixed Earth, with a small perpendicular
    /// offset so the nominal is a *conditioned* hit (a fast pass — 20 km/s — so
    /// the osculating hyperbola at CA stays well above Earth escape speed and
    /// `b_plane` is valid). A cross-track impulse opens the miss linearly, so the
    /// solver's bracket-expand + bisect and its perigee objective are checked
    /// against known behavior.
    fn straight_line_scenario<'a>(
        force: &'a ZeroForce,
        earth: &'a (dyn GeocentricState + 'a),
    ) -> DeflectionScenario<'a> {
        let x0 = 1.0e10; // start 0.067 AU back along −x
        let b0 = 4.0e6; // 4 000 km perpendicular offset → conditioned nominal hit
        let v = 20_000.0; // 20 km/s closing speed (> v_esc keeps the pass hyperbolic)
        let seed = StateVector::from_components(-x0, b0, 0.0, v, 0.0, 0.0);
        let scan = ScanOptions {
            max_sample_dt: 1_800.0,
            time_tol_seconds: 1.0e-3,
            max_distance: Some(5.0e8),
        };
        // CA is at t ≈ x0/v = 5e5 s; propagate to 1e6 s so it is well inside.
        DeflectionScenario::new(
            Dop853::new(),
            force,
            earth,
            Epoch::from_tdb_seconds_past_j2000(0.0),
            seed,
            1.0e4,
            100,
            scan,
            MU_EARTH,
            R_EARTH,
        )
        .expect("scenario builds")
    }

    #[test]
    fn nominal_straight_line_pass_is_a_conditioned_hit() {
        let force = ZeroForce;
        let earth = |_e: Epoch| Ok(StateVector::new(Vector3::zeros(), Vector3::zeros()));
        let sc = straight_line_scenario(&force, &earth);

        let enc = sc
            .nominal_encounter()
            .expect("nominal reduces")
            .expect("a close approach exists");
        assert!(enc.is_hit(), "nominal must be a hit (perigee ≤ capture)");
        // The pass grazes the 4 000 km perpendicular offset, so the osculating
        // hyperbola's perigee ≈ that offset; the impact parameter is the (larger)
        // focused asymptotic offset above it.
        assert!(
            (enc.perigee - 4.0e6).abs() < 1.0e5,
            "perigee ≈ 4000 km, got {:.3e}",
            enc.perigee
        );
        assert!(enc.impact_parameter > enc.perigee);
        assert!(enc.perigee < enc.capture_radius);
    }

    #[test]
    fn required_cross_track_dv_hits_target_perigee_and_is_monotone() {
        let force = ZeroForce;
        let earth = |_e: Epoch| Ok(StateVector::new(Vector3::zeros(), Vector3::zeros()));
        let sc = straight_line_scenario(&force, &earth);

        let e0 = Epoch::from_tdb_seconds_past_j2000(0.0);
        let cross = Vector3::new(0.0, 1.0, 0.0);
        let tol = DvSolveTol::default();

        // Solve for a 15 000 km perigee (a clean miss, above the capture radius).
        let target = 1.5e7;
        let dv = sc
            .required_dv(e0, cross, target, tol)
            .expect("solver converges");
        assert!(dv > 0.0);

        // The solution actually delivers the requested perigee.
        let achieved = sc
            .evaluate(e0, dv * cross)
            .expect("re-eval")
            .expect("still an encounter")
            .perigee;
        assert!(
            (achieved - target).abs() <= 5.0e-3 * target,
            "achieved perigee {achieved:.4e} m vs target {target:.4e} m"
        );

        // A larger safe miss must cost more Δv.
        let dv_far = sc
            .required_dv(e0, cross, 3.0e7, tol)
            .expect("solver converges");
        assert!(
            dv_far > dv,
            "a bigger required miss must need a bigger Δv ({dv_far:.4e} vs {dv:.4e})"
        );
    }

    #[test]
    fn no_impulse_needed_when_nominal_already_clears_target() {
        let force = ZeroForce;
        let earth = |_e: Epoch| Ok(StateVector::new(Vector3::zeros(), Vector3::zeros()));
        let sc = straight_line_scenario(&force, &earth);

        // Nominal perigee is ~4000 km; asking for a target *below* that needs no
        // deflection, so the solver returns exactly 0.
        let dv = sc
            .required_dv(
                Epoch::from_tdb_seconds_past_j2000(0.0),
                Vector3::new(0.0, 1.0, 0.0),
                1.0e6,
                DvSolveTol::default(),
            )
            .expect("solve");
        assert_eq!(dv, 0.0);
    }

    /// `deflected_trajectory` and `evaluate` must never disagree on the encounter:
    /// `evaluate` is *defined* as `deflected_trajectory().1`, and the animation
    /// draws the returned clock while the planner reports that same perigee — so
    /// the visible track and the headline number come from one propagation. We
    /// also re-scan the returned clock ourselves and confirm the perigee it yields
    /// matches, i.e. the sampled arc the painter walks is the arc the b-plane was
    /// reduced from — the displayed miss cannot silently lie.
    #[test]
    fn deflected_trajectory_and_evaluate_agree() {
        let force = ZeroForce;
        let earth = |_e: Epoch| Ok(StateVector::new(Vector3::zeros(), Vector3::zeros()));
        let sc = straight_line_scenario(&force, &earth);

        let e0 = Epoch::from_tdb_seconds_past_j2000(0.0);
        // A cross-track nudge that opens a real miss (not a clean escape past the
        // scan gate, so both paths carry a Some(encounter) to compare).
        let dv = Vector3::new(0.0, 0.05, 0.0);

        let from_eval = sc.evaluate(e0, dv).expect("evaluate");
        let (clock, from_traj) = sc.deflected_trajectory(e0, dv).expect("trajectory");

        // Both must agree the pass is (or isn't) an encounter, and on its perigee.
        match (from_eval, from_traj) {
            (Some(a), Some(b)) => {
                assert_eq!(
                    a.perigee, b.perigee,
                    "evaluate and deflected_trajectory disagree on perigee"
                );
                // The returned clock, re-scanned independently, reproduces it.
                let ca = closest_approach(&clock, &earth, sc.scan)
                    .expect("scan")
                    .expect("close approach");
                let bp = ca.b_plane(MU_EARTH, R_EARTH).expect("b-plane");
                assert!(
                    (bp.perigee - a.perigee).abs() <= 1.0e-6 * a.perigee.abs().max(1.0),
                    "re-scanned clock perigee {:.6e} ≠ reported {:.6e}",
                    bp.perigee,
                    a.perigee
                );
            }
            (None, None) => panic!("expected an encounter for this nudge"),
            (l, r) => panic!("evaluate/trajectory encounter mismatch: {l:?} vs {r:?}"),
        }

        // The clock is the post-deflection arc: it starts at the deflection epoch
        // with the impulse folded into the seed velocity.
        let seed = sc.nominal().state_at(e0).unwrap();
        let start = clock.state_at(e0).unwrap();
        assert_eq!(start.velocity, seed.velocity + dv);
    }

    // ---- Test 3: the thesis — earlier deflection costs less Δv ---------------

    /// The *direction* of the headline lesson (§1, §5, HANDOFF line 144): on a
    /// heliocentric orbit, an along-track nudge applied earlier is more effective,
    /// so it takes **less** Δv to clear the same target. This is a genuinely
    /// orbital effect — an along-track kick opens a miss only through the changed
    /// orbit; in straight-line drift it would open none.
    ///
    /// Scope caveat: both leads here are **sub-orbital** (0.7 vs 0.1 period), so
    /// this pins the sign of the leverage over a single arc — it does **not**
    /// exercise the multi-orbit timing accumulation that produces the dramatic
    /// `Δv ∝ 1/lead` falloff (that needs many revolutions). The steeper-than-
    /// linear falloff is validated by the real-field headline curve in the viewer
    /// (§10 task 10), not asserted here. Kernel-free: a Sun-only two-body asteroid
    /// on a direct-impact course toward a fixed (fast-relative) Earth point.
    #[test]
    fn earlier_along_track_deflection_needs_less_dv() {
        let force = sun_only();

        // A ~1.5 yr heliocentric orbit seeded at 1 AU on the +x axis.
        let e0 = Epoch::from_tdb_seconds_past_j2000(0.0);
        let seed = StateVector::from_components(AU_M, 0.0, 0.0, 0.0, 33_000.0, 3_000.0);

        // Orbital period from vis-viva: T = 2π√(a³/μ).
        let r = seed.position.norm();
        let v2 = seed.velocity.norm_squared();
        let a = 1.0 / (2.0 / r - v2 / MU_SUN);
        let period = std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt();

        let cadence = 86_400.0; // 1 day
        let t_end = 0.8 * period;
        let n = (t_end / cadence).ceil() as u32;

        // Propagate the nominal once (Earth-free) to locate the impact point:
        // the asteroid's own position at t_c, nudged perpendicular to its
        // velocity by 5000 km so the nominal is a conditioned hit.
        let probe = Clock::propagate(&Dop853::new(), &force, e0, seed, cadence, n).unwrap();
        let t_c = 0.7 * period;
        let epoch_c = Epoch::from_tdb_seconds_past_j2000(t_c);
        let ast_c = probe.state_at(epoch_c).unwrap();
        let perp = {
            let p = ast_c.velocity.cross(&Vector3::z());
            p / p.norm()
        };
        let earth_pos = ast_c.position + 5.0e6 * perp;
        let earth = move |_e: Epoch| Ok(StateVector::new(earth_pos, Vector3::zeros()));

        let scan = ScanOptions {
            max_sample_dt: 6.0 * 3600.0,
            time_tol_seconds: 1.0e-3,
            max_distance: Some(5.0e8),
        };
        let sc = DeflectionScenario::new(
            Dop853::new(),
            &force,
            &earth,
            e0,
            seed,
            cadence,
            n,
            scan,
            MU_EARTH,
            R_EARTH,
        )
        .expect("scenario builds");

        // The nominal is the impact we are trying to undo.
        let enc = sc.nominal_encounter().unwrap().expect("an encounter");
        assert!(
            enc.is_hit(),
            "nominal must be a hit; perigee {:.3e}",
            enc.perigee
        );

        let target = 1.2e7; // 12 000 km — above the capture radius → a safe miss
        let tol = DvSolveTol::default();

        // Deflect early (lead ≈ 0.7 period) vs late (lead ≈ 0.1 period). The
        // leverage gap over sub-orbital leads is modest — the dramatic 1/lead
        // scaling is a many-orbit asymptotic — so the contrast is drawn between a
        // long and a short lead rather than two nearby ones.
        let dv_early = sc
            .required_dv_along_track(e0, target, tol)
            .expect("early solve");
        let epoch_late = Epoch::from_tdb_seconds_past_j2000(0.6 * period);
        let dv_late = sc
            .required_dv_along_track(epoch_late, target, tol)
            .expect("late solve");

        assert!(dv_early > 0.0 && dv_late > 0.0);
        assert!(
            dv_early < 0.75 * dv_late,
            "the thesis: earlier deflection must cost meaningfully less Δv \
             (early {dv_early:.4e} m/s vs late {dv_late:.4e} m/s)"
        );
    }

    // --- Nuclear standoff (§5) -----------------------------------------------
    //
    // The published bodies these tests quote, in one place so no test restates
    // them. Both come from the LLNL papers cited on `StandoffNuclear`; neither is
    // a body this project invented.

    /// UCRL-PROC-228569's "standard structure": 1000 m diameter, 1.05e12 kg.
    const DEARBORN_BODY_MASS_KG: f64 = 1.05e12;
    const DEARBORN_BODY_RADIUS_M: f64 = 500.0;
    /// LLNL-PROC-485160's Apophis-sized non-porous model: 270 m across,
    /// "27.8 million tons".
    const APOPHIS_SIZED_MASS_KG: f64 = 2.78e10;
    const APOPHIS_SIZED_RADIUS_M: f64 = 135.0;

    /// The fixture the coefficient was derived from: 100 kt at the optimum height
    /// of burst over the 1 km body gives the paper's 6.5 mm/s.
    ///
    /// This is a **self-consistency** check, not independent validation — the
    /// coupling constant was inverted from this very case. Its job is to catch a
    /// typo'd constant or a units slip (kt↔J, mm/s↔m/s), and to make the
    /// provenance executable rather than only documented.
    #[test]
    fn standoff_nuclear_reproduces_the_published_nudge_model() {
        let dv = StandoffNuclear::DEARBORN_2007
            .dv_ms(100.0, DEARBORN_BODY_MASS_KG)
            .expect("valid inputs");
        assert!(
            (dv - 6.5e-3).abs() <= 0.01 * 6.5e-3,
            "UCRL-PROC-228569 Nudge Model: 100 kt at d/R = 0.6 over a 1.05e12 kg \
             body settles at ≈6.5 mm/s; got {:.4} mm/s",
            dv * 1e3
        );

        // And the escape speed the paper states independently (≈0.5 m/s) must fall
        // out of its own stated mass and diameter — the check that 1 km is the
        // body's *diameter*, not its radius. Reading it the other way would make
        // every Δv in this module wrong by ~8x.
        let v_esc = escape_speed_ms(DEARBORN_BODY_MASS_KG, DEARBORN_BODY_RADIUS_M).expect("v_esc");
        assert!(
            (v_esc - 0.5).abs() < 0.05,
            "the paper's own ≈0.5 m/s escape speed pins 1 km as the diameter; got {v_esc:.4} m/s"
        );
    }

    /// The coefficient must stay inside the band the published series define — a
    /// guard on *provenance*, which no numerical test of the model itself would
    /// catch, and the only check here that bounds the coefficient from **above**.
    ///
    /// LLNL-PROC-485160 reports a separate run: 10 kt deposited ablates ~4000 t at
    /// "over 2 km/s", i.e. `C_m ≥ 1.91e-4 N·s/J`. Note that is a *lower bound*, so
    /// it does not bracket the shipped 1.418e-4 — both point the same way and the
    /// 36 % gap is a floor. Nothing forces two different simulation series to
    /// agree; if a future edit moves the shipped constant outside this band it has
    /// left the published range and needs a new citation.
    #[test]
    fn momentum_coupling_stays_in_the_published_band() {
        let ours = StandoffNuclear::DEARBORN_2007.momentum_coupling_ns_per_j;
        let from_2011 = 4.0e6 * 2000.0 / (10.0 * JOULES_PER_KILOTONNE);

        assert!(
            (from_2011 - 1.91e-4).abs() < 1.0e-6,
            "the 2011 figure recomputes to 1.91e-4 N·s/J; got {from_2011:.4e}"
        );
        let spread = (from_2011 - ours).abs() / ours;
        assert!(
            spread < 0.40,
            "the two published series must stay within well under a factor of two \
             (ours {ours:.4e} vs 2011 {from_2011:.4e} = {:.0} %)",
            spread * 100.0
        );
        // Ours is the smaller, and the direction matters twice over. A smaller
        // momentum coupling means *more* yield is required for a given Δv, so the
        // requirement errs high rather than flattering itself. And because the
        // 2011 value is a lower bound, this inequality is the only thing in the
        // suite bounding the coefficient from above at all — the fragmentation
        // test bounds it from below, and nothing else touches it.
        assert!(
            ours < from_2011,
            "the shipped coefficient should be the conservative (smaller) value"
        );
    }

    /// **The independent check.** A different paper, a different body, a different
    /// yield, and a case this coefficient was never fitted to: LLNL-PROC-485160
    /// deposits 17 kt into a 270 m, 2.78e10 kg non-porous body and reports it
    /// "completely fragmented" (Ke/Pe > 1).
    ///
    /// The model must independently land that case above the escape speed — the
    /// assertion that fails if the coefficient is wrong by the order of magnitude
    /// a dimensional-plausibility check would happily let through.
    ///
    /// # What this test does *not* bound
    /// It is **one-sided**, in two compounding ways. It fails if `C_m` is ~2.19×
    /// too *low* and never if it is too high; the upper bound comes only from
    /// [`momentum_coupling_stays_in_the_published_band`]. And it extrapolates the
    /// coefficient across roughly **20× the surface fluence** it was fitted at
    /// (17 kt over a 135 m radius here, against 11.5 kt over 500 m), in the
    /// direction where momentum coupling per joule is known to *fall*. So the
    /// 2.19 carries perhaps a factor of two of margin against a real systematic
    /// rather than being a tight prediction — which is enough for the qualitative
    /// claim being tested (above escape speed, therefore disrupted) and not enough
    /// for anything quantitative.
    #[test]
    fn standoff_nuclear_predicts_the_published_fragmentation_case() {
        // The paper states energy *absorbed*, not device yield, so the coupling
        // factor is bypassed rather than guessed at (η = 1 with the same C_m).
        let absorbed_only = StandoffNuclear {
            coupling_efficiency: 1.0,
            ..StandoffNuclear::DEARBORN_2007
        };
        let dv = absorbed_only
            .dv_ms(17.0, APOPHIS_SIZED_MASS_KG)
            .expect("valid inputs");
        let v_esc = escape_speed_ms(APOPHIS_SIZED_MASS_KG, APOPHIS_SIZED_RADIUS_M).expect("v_esc");
        let ratio = dv / v_esc;

        println!(
            "17 kt absorbed into the 270 m body: Δv = {:.3} m/s vs v_esc = {:.3} m/s \
             → Δv/v_esc = {ratio:.2}",
            dv, v_esc
        );
        assert!(
            ratio > 1.0,
            "the paper reports this case completely fragmented (Ke/Pe > 1), so the \
             model must put Δv above the escape speed; got Δv/v_esc = {ratio:.2}"
        );
        assert_eq!(
            disruption_regime(dv, APOPHIS_SIZED_MASS_KG, APOPHIS_SIZED_RADIUS_M),
            Some(DisruptionRegime::LikelyDisruption)
        );
    }

    /// The other end of the same classifier: the Nudge Model kept 99 % of the mass
    /// bound, so its own Δv must read as intact. Both anchors are asserted, so the
    /// band between them cannot quietly swallow either.
    #[test]
    fn the_nudge_model_case_is_classified_intact() {
        let dv = StandoffNuclear::DEARBORN_2007
            .dv_ms(100.0, DEARBORN_BODY_MASS_KG)
            .expect("valid inputs");
        assert_eq!(
            disruption_regime(dv, DEARBORN_BODY_MASS_KG, DEARBORN_BODY_RADIUS_M),
            Some(DisruptionRegime::IntactDeflection)
        );

        // And the uncharacterised band is genuinely reachable — a three-state enum
        // whose middle state no input produces is a two-state enum with a lie in it.
        let v_esc = escape_speed_ms(DEARBORN_BODY_MASS_KG, DEARBORN_BODY_RADIUS_M).expect("v_esc");
        let between = 0.5 * (INTACT_DV_OVER_VESC + DISRUPTION_DV_OVER_VESC) * v_esc;
        assert_eq!(
            disruption_regime(between, DEARBORN_BODY_MASS_KG, DEARBORN_BODY_RADIUS_M),
            Some(DisruptionRegime::Uncharacterised)
        );
    }

    /// Both inverts are exact, so both round-trip. This is what lets the headline
    /// Δv curve be requoted in kilotonnes and kilograms with no extra propagation.
    #[test]
    fn method_inverts_round_trip() {
        let nuke = StandoffNuclear::DEARBORN_2007;
        let y = nuke
            .yield_kilotonnes_for_dv(6.5e-3, DEARBORN_BODY_MASS_KG)
            .expect("invert");
        assert!(
            (y - 100.0).abs() < 1.0,
            "6.5 mm/s on the 1 km body must invert back to ≈100 kt; got {y:.3} kt"
        );
        let back = nuke.dv_ms(y, DEARBORN_BODY_MASS_KG).expect("forward");
        assert!((back - 6.5e-3).abs() < 1.0e-9);

        let m = kinetic_impactor_mass_for_dv(6.5e-3, 3.6, 1.0e4, DEARBORN_BODY_MASS_KG)
            .expect("invert");
        let dv_back = kinetic_impactor_dv(3.6, m, 1.0e4, DEARBORN_BODY_MASS_KG);
        assert!((dv_back - 6.5e-3).abs() < 1.0e-12);
    }

    /// **The comparison, at one bar.** The point of adding a second method is not
    /// a second formula — it is that the two can be quoted against the *same* Δv,
    /// on the *same* body, and read side by side. Three methods quoted against
    /// three different bars would look comparable and would not be.
    ///
    /// Everything here is either published or explicitly stated: the body is
    /// UCRL-PROC-228569's, the required Δv is that paper's own 6.5 mm/s, and the
    /// interception geometry (β = 3.6 from DART, 10 km/s relative speed) is a
    /// stated assumption rather than a derived result — labelled as such, because
    /// the ratio depends on it.
    ///
    /// # This table does not transfer to the campaign's own threat
    /// It concludes on a **1.05e12 kg** body, and its `IntactDeflection` verdict is
    /// a property of *that* rock's 0.53 m/s escape speed. The shipping threat is
    /// 2.83e10 kg with an escape speed of 0.159 m/s, where the same comparison
    /// comes out the other way — every lead the campaign covers needs a Δv above
    /// escape, i.e. disruption rather than deflection. Core must not learn about
    /// that body, so the on-threat table lives in the binding as
    /// `deflection_methods_compared_at_one_bar_on_the_real_threat`, and **it is
    /// the one the frontend should ever quote**. Reading this one as the
    /// campaign's answer is precisely the mistake the J2 pair already caught once.
    #[test]
    fn deflection_methods_compared_at_one_bar() {
        let dv_required = 6.5e-3_f64;
        let beta = 3.6_f64;
        let v_rel = 1.0e4_f64;

        let mass = kinetic_impactor_mass_for_dv(dv_required, beta, v_rel, DEARBORN_BODY_MASS_KG)
            .expect("kinetic invert");
        let yield_kt = StandoffNuclear::DEARBORN_2007
            .yield_kilotonnes_for_dv(dv_required, DEARBORN_BODY_MASS_KG)
            .expect("nuclear invert");

        // 14 714 kg is the heaviest mass any vehicle in `launch_vehicle.rs`
        // delivers at its cheapest C3 (Falcon Heavy expendable) — the same figure
        // the porkchop layer seeds its mass solve from. Quoted, not recomputed,
        // because that table lives above this crate.
        const HEAVIEST_DELIVERABLE_KG: f64 = 14_714.0;
        let launches = mass / HEAVIEST_DELIVERABLE_KG;

        println!(
            "one bar — Δv = {:.2} mm/s on a {:.2e} kg body:\n  \
             kinetic impactor (β = {beta}, v_rel = {:.0} km/s): {mass:.0} kg = {launches:.1} × \
             the heaviest single launch\n  \
             standoff nuclear: {yield_kt:.1} kt",
            dv_required * 1e3,
            DEARBORN_BODY_MASS_KG,
            v_rel / 1e3
        );

        assert!(
            launches > 10.0,
            "the lesson: against a 1 km body the kinetic route needs many launches \
             ({launches:.1} here), which is why §5 puts a nuclear term at the other \
             end of the spectrum"
        );
        assert!(
            (yield_kt - 100.0).abs() < 1.0,
            "and the nuclear route asks for one device of a stated, published class \
             ({yield_kt:.1} kt)"
        );

        // The comparison is only honest if the nuclear option is also *survivable*
        // at this bar — a bigger Δv that shatters the body is not a better answer.
        assert_eq!(
            disruption_regime(dv_required, DEARBORN_BODY_MASS_KG, DEARBORN_BODY_RADIUS_M),
            Some(DisruptionRegime::IntactDeflection)
        );
    }

    /// Invalid inputs return `None` rather than a `NaN` that would poison a display
    /// (the porkchop layer's sentinel-not-`NaN` rule, applied one module earlier).
    #[test]
    fn nuclear_rejects_degenerate_inputs() {
        let nuke = StandoffNuclear::DEARBORN_2007;
        assert_eq!(nuke.dv_ms(100.0, 0.0), None);
        assert_eq!(nuke.dv_ms(f64::NAN, 1.0e12), None);
        assert_eq!(nuke.dv_ms(-1.0, 1.0e12), None);
        assert_eq!(nuke.yield_kilotonnes_for_dv(1.0, -1.0), None);
        assert_eq!(escape_speed_ms(1.0e12, 0.0), None);
        assert_eq!(disruption_regime(1.0, 0.0, 100.0), None);

        let bad = StandoffNuclear {
            coupling_efficiency: 1.5,
            momentum_coupling_ns_per_j: 1.418e-4,
        };
        assert_eq!(bad.dv_ms(100.0, 1.0e12), None, "η > 1 is not a coupling");
    }
}
