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
use nalgebra::{Vector2, Vector3};

use crate::ephemeris::{Ephemeris, EphemerisError, KM3_S2_TO_M3_S2};
use crate::forces::oblateness::Oblateness;
use crate::forces::relativity::Relativity1PN;
use crate::forces::srp::SolarRadiationPressure;
use crate::forces::yarkovsky::YarkovskyA2;
use crate::forces::CompositeForce;
use crate::geometry::BPlaneEncounter;
use crate::perturber_field::{
    pluto_perturber_field, sb441_perturber_field, tier1_perturber_field, EphemerisPerturber,
    EphemerisPole,
};
use crate::uncertainty::{
    bplane_jacobian, BPlaneBasis, BPlaneSensitivity, BPlaneUncertainty, LinearityReport,
    StateCovariance, UncertaintyError, SAMPLE_CADENCE_DAYS,
};
use crate::{
    find_close_approaches, geometry, Clock, DeflectionError, DeflectionScenario, Dop853, DvSolveTol,
    Epoch, Integrator, OrbitalElements, ScanOptions, StateVector,
};

/// Metres per kilometre — the km→m scale the DE440 states cross into SI on.
const KM_TO_M: f64 = 1.0e3;
/// Seconds in a Julian year (365.25 d), for lead-time bookkeeping.
const SECONDS_PER_YEAR: f64 = 365.25 * 86_400.0;

/// How long before the nominal closest approach every Tier-3 uncertainty sample
/// is reduced to b-plane geometry.
///
/// Twelve hours puts this campaign's rock ~330 000 km out at its 7.6 km/s `v_inf`:
/// inside Earth's ~924 000 km sphere of influence, so the osculating hyperbola
/// really is the encounter; far outside the well, so `v_inf = √(v² − 2μ/r)` is not
/// a subtraction of near-equal squares; and inside the close-approach scan gate.
/// See [`RealFieldScenario::uncertainty_sample`] for why the epoch is fixed at all.
pub const UNCERTAINTY_REDUCTION_LEAD_SECONDS: f64 = 12.0 * 3600.0;

/// Half-width of the encounter window the animation samples, seconds. ±1.5 days
/// brackets the fast (18 km/s) pass with room for a modestly time-shifted
/// deflected closest approach. Shared by renderers and their tests so a test
/// exercises the resolution the app actually renders.
pub const ENCOUNTER_HALF_WINDOW_SECONDS: f64 = 1.5 * 86_400.0;
/// Samples across the encounter window — dense enough that the track is smooth
/// through the tight turn near closest approach.
pub const ENCOUNTER_SAMPLES: usize = 1_400;

/// The campaign's **safe-perigee target**, metres — 20 000 km, ≈ 3.13 `R⊕`.
///
/// The one number every "how much deflection is enough" answer in this project is
/// measured against: the headline Δv-vs-lead curve solves for the along-track
/// impulse that lifts the b-plane perigee to *this* (`viewer/src/bin/curve.rs`, and
/// the `target_perigee_m` recorded in the `curve.json` it writes), and the
/// launch-window map's required-impactor-mass solve targets the same value. Naming
/// it once is what lets those two compose: a mass requirement and a Δv requirement
/// quoted against different targets would look comparable and not be.
///
/// **A margin, not a hit test.** The verdict question — did this pass hit Earth — is
/// `|B|` against `b_capture` (equivalently, perigee against `R⊕`), and mixing those
/// pairs is a bug this project shipped once. This is a different question: a design
/// goal, stated in the perigee's own units, deliberately clear of the focused
/// capture disc (~11 311 km for the shipping nominal) rather than grazing it. Any
/// readout quoting a requirement solved against this **must name the target**, or it
/// reads as "the mass needed to miss Earth", which is a smaller number.
pub const SAFE_PERIGEE_TARGET_M: f64 = 2.0e7;

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

/// Earth's `J2` b-plane perigee shift measured on a genuine **miss** geometry, km,
/// signed the way the frontend menu signs every shift (`baseline − shifted`, so
/// positive = the term pulls the perigee *inward*).
///
/// The companion to the `J2` figure the force-model menu shows. Every term in that
/// menu is measured on the shipping nominal — which it must be, since they are all
/// differenced against the same baseline — but that nominal is a designed **impact**
/// whose closest approach is 3000 km, *inside* Earth, and the `J2` expansion is only
/// valid outside `R_eq`. So `J2` alone among the five is measured out of its own
/// domain there. This is the same term on the geometry that actually matters: a
/// deflected pass, along-track, one year before impact, whose perigee lands at
/// **3.0 `R_eq`** and whose `|B|` clears the capture disc — a clean miss (the
/// impulse and the geometry it reaches are named in
/// `earth_j2_on_a_deflected_miss_is_in_domain`, which measures this and would fail
/// if the constant drifted from what the physics says).
///
/// Measured: **−0.1196 km** — `J2` eases that pass ~120 m *outward*, where the same
/// term on the impact geometry shows +1.33 km *inward*. Different magnitude and
/// different sign, and the sign is not by itself evidence of the domain problem:
/// the term carries a Legendre factor in the latitude of closest approach, which
/// two different passes have no reason to share. What the two numbers establish is
/// the narrower, sufficient point — the menu's 1.33 km is *this geometry's* number
/// and not "what `J2` does to a deflection", which is why the panel captions it.
/// The domain claim is carried by the capture-radius bias, which collapses ~480×
/// between them (0.687 % → 0.0014 %), tracking the `1/r³` its mechanism predicts.
///
/// A recorded constant rather than something computed at run time: it costs a pair
/// of full propagations, it never changes for a given scenario, and the frontend
/// needs it to caption a number rather than to fly anything.
pub const J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM: f64 = -0.1196;

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
    /// Enable Earth's `J2` oblateness
    /// ([`Oblateness`](crate::forces::oblateness::Oblateness)) with the DE440
    /// `J2E`/`RE` pair and the spin axis ANISE rotates out of the loaded
    /// orientation data (never `ẑ` assumed).
    ///
    /// `J2` falls off as `1/r⁴`, so along the heliocentric cruise it is nothing;
    /// it exists for the **close Earth flyby**, where the asteroid spends minutes
    /// inside a few Earth radii. Expect the b-plane shift to be dominated entirely
    /// by that final pass — and expect it to be small. Report it, do not amplify it.
    pub earth_j2: bool,
    /// Enable **Pluto** as an 11th point-mass perturber
    /// ([`pluto_perturber_field`](crate::perturber_field::pluto_perturber_field)) —
    /// the one body ASSIST's point-mass term carries that §5's locked "Sun + 8
    /// planets + Moon" shipping set omits (HANDOFF open questions).
    ///
    /// Off by [`Default`] like every other Tier-2 term, so the shipping demo stays
    /// the ten-body field it has always been. Whether Pluto belongs *in* that set
    /// is the measured question the batch-2c ~55 m-over-two-years figure left open;
    /// the fixed-seed b-plane comparison below answers it at the campaign's real
    /// lead time.
    pub pluto: bool,
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
    /// sub-cadence queries, so *between* snapshots this costs nothing.
    ///
    /// **It is not free across them, contrary to what this said until it was
    /// measured.** [`Clock::propagate`] restarts the adaptive integrator at every
    /// snapshot, so the cadence sets the step-size regime as well as the storage:
    /// `probe_tier3_cost` finds the b-plane perigee moves **+3 cm at 3 days,
    /// +118 m at 10, and +13.6 km at 30** against the shipping 1 day. The shipping
    /// value is the finest of those and nothing built on it is affected — but a
    /// caller that coarsens the cadence for speed is buying that speed with
    /// accuracy, not just with memory. Derivatives are far more forgiving than
    /// absolute positions here (a difference of two runs at one cadence cancels the
    /// systematic error), which is why [`SAMPLE_CADENCE_DAYS`] can afford 10 days
    /// where this cannot.
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

/// The unit vector the designed impact offset is laid along: perpendicular to the
/// relative velocity.
///
/// Extracted rather than written twice because [`RealFieldScenario::build_with`]
/// and [`ImpactorConfig::preview`] must place the impact point at the *same*
/// spot — a preview that quotes a miss distance for a geometry the builder does
/// not construct is worse than no preview. The seed-axis switch avoids a
/// near-parallel cross product (which would lose precision, then normalize the
/// noise back up to unit length) without caring which perpendicular it lands on:
/// the offset direction within the b-plane is arbitrary by construction, only its
/// magnitude and its perpendicularity to `vdir` carry meaning.
///
/// **`r_rel ⊥ v_rel` is the load-bearing consequence.** It makes the designed
/// impact point the perigee of the geocentric hyperbola, which is what lets
/// `preview` reach the incoming asymptote in closed form.
fn impact_offset_axis(vdir: &Vector3<f64>) -> Vector3<f64> {
    let seed_axis = if vdir.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let p = vdir.cross(&seed_axis);
    p / p.norm()
}

/// What an [`ImpactorConfig`]'s encounter geometry and heliocentric orbit look
/// like — computed in **closed form**, without the multi-year back-propagation
/// [`RealFieldScenario::build_with`] costs.
///
/// See [`ImpactorConfig::preview`] for what it is and is not good for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreatOrbitPreview {
    /// Hyperbolic excess speed about Earth, m/s. **Not** `cfg.v_rel_kms`, which
    /// is the speed at the impact point deep in Earth's well; this is what is
    /// left after climbing out, and it is what sets the capture radius.
    pub v_inf: f64,
    /// b-plane impact parameter, m — the perpendicular miss of the *incoming
    /// asymptote* from Earth's centre.
    ///
    /// **Also not `cfg.b_offset_km`.** Focusing means the asymptote passes wider
    /// than the point it is aimed through: `b = b_offset · v_rel / v_inf`, which
    /// at the shipping config is 7 077 km for a 3 000 km offset.
    pub impact_parameter: f64,
    /// Earth's focused collision disc, m — the bar `impact_parameter` is measured
    /// against.
    pub capture_radius: f64,
    /// Whether this geometry is still a designed *impact*. `false` means
    /// [`RealFieldScenario::build_with`] would reject it with
    /// [`ScenarioError::NominalNotAHit`] — after paying the full back-propagation.
    /// Reported rather than raised so a live UI can show the two numbers that
    /// disagree instead of just refusing.
    pub is_hit: bool,
    /// Heliocentric semi-major axis of the *incoming* orbit, m.
    pub semi_major_axis_m: f64,
    /// Heliocentric eccentricity of the incoming orbit.
    pub eccentricity: f64,
    /// Heliocentric inclination of the incoming orbit, radians.
    pub inclination_rad: f64,
    /// Heliocentric orbital period of the incoming orbit, seconds — the unit the
    /// deflection curve's "lead in orbits" is counted in.
    pub period_seconds: f64,
}

impl ImpactorConfig {
    /// The encounter geometry and heliocentric orbit this config implies, in
    /// closed form — **microseconds**, against the ~10 s
    /// [`RealFieldScenario::build_with`] costs.
    ///
    /// # Why this is possible at all
    /// The designed impact places the offset perpendicular to the relative
    /// velocity (see [`impact_offset_axis`]), so the impact point *is* the perigee
    /// of the geocentric hyperbola. [`BPlaneEncounter::from_relative_state`] then
    /// reduces that state to `v_inf` and the incoming asymptote `Ŝ` with no
    /// integration, and the incoming heliocentric velocity is just
    /// `v_earth + v_inf·Ŝ` — the flyby undone analytically instead of numerically.
    ///
    /// # How close it is — measured, in two very different halves
    /// `preview_tracks_the_built_orbit` differences this against real builds across
    /// the range the frontend's knobs reach (0.68–2.66 yr of period):
    ///
    /// - **The encounter geometry is exact.** `v_inf` and `impact_parameter` match
    ///   the propagated nominal's own b-plane reduction to **0.001 %**. That is not
    ///   a tolerance being met, it is the same closed form arriving at the same
    ///   answer — the builder designs this state and the integrator hands it back.
    /// - **The orbit is an estimate.** `a`/`period` are osculating at the **impact
    ///   epoch**, where `build_with` reports vis-viva at the **seed**, `lead_years`
    ///   earlier, after a real integration through the perturbed field. Worst
    ///   observed gap **0.23 %** (the steep approach); even the long-period
    ///   prograde extreme, where `v_inf` lies along Earth's 29.8 km/s and `a` is
    ///   most sensitive to it, holds to 0.155 %.
    ///
    /// 0.23 % is good enough to *label* a knob and not good enough to *score* a
    /// plan: the tractor bench divides the lead by the period, so it must keep
    /// taking `period_seconds()` from the built scenario — which costs nothing,
    /// because by then the rebuild has already landed.
    ///
    /// # Errors
    /// [`ScenarioError::ImpactNotHyperbolic`] if the relative speed is too low for
    /// the offset to be a flyby at all (Earth escape at 3 000 km is 16.3 km/s, so
    /// this wall is *close* to the shipping 18 km/s and gets closer as the offset
    /// shrinks), and [`ScenarioError::UnboundOrbit`] if the resulting heliocentric
    /// orbit is not an ellipse. A geometry that merely stops being a hit is
    /// reported through [`ThreatOrbitPreview::is_hit`], not raised.
    pub fn preview(&self, eph: &Arc<Ephemeris>) -> Result<ThreatOrbitPreview, ScenarioError> {
        let earth = EphemerisPerturber::new(Arc::clone(eph), EARTH_J2000);
        let sun = EphemerisPerturber::new(Arc::clone(eph), SUN_J2000);
        let mu_earth = eph
            .gm_km3_s2(EARTH_J2000)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            * KM3_S2_TO_M3_S2;
        let mu_sun = eph
            .gm_km3_s2(SUN_J2000)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            * KM3_S2_TO_M3_S2;

        // The same impact state `build_with` designs, through the same axis helper.
        let vdir = self.v_rel_dir.normalize();
        let r_rel = self.b_offset_km * KM_TO_M * impact_offset_axis(&vdir);
        let v_rel = self.v_rel_kms * KM_TO_M * vdir;

        let enc = BPlaneEncounter::from_relative_state(
            r_rel,
            v_rel,
            mu_earth,
            geometry::EARTH_EQUATORIAL_RADIUS_M,
        )
        .map_err(|e| {
            ScenarioError::ImpactNotHyperbolic(format!(
                "{e} — v_rel = {:.3} km/s at a {:.0} km offset is not a flyby \
                 (Earth escape there is {:.3} km/s)",
                self.v_rel_kms,
                self.b_offset_km,
                (2.0 * mu_earth / (self.b_offset_km * KM_TO_M)).sqrt() / KM_TO_M,
            ))
        })?;

        // Undo the flyby: far before the encounter the body moved at `v_inf` along
        // the incoming asymptote, so its heliocentric velocity was Earth's plus
        // that. Position is Earth's — a few thousand km against 1 AU.
        let earth_impact = earth
            .state_at(self.impact_epoch)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let sun_impact = sun
            .state_at(self.impact_epoch)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let helio = StateVector::new(
            earth_impact.position + r_rel - sun_impact.position,
            earth_impact.velocity + enc.v_inf * enc.s_hat - sun_impact.velocity,
        );

        let el = OrbitalElements::from_state(helio, mu_sun).map_err(|_| {
            // Both element failures mean the same thing for a knob: there is no
            // period, so "lead in orbits" has no meaning. Recompute `a` by vis-viva
            // to carry the number the error's message quotes.
            let r = helio.position.norm();
            let v2 = helio.velocity.norm_squared();
            ScenarioError::UnboundOrbit(1.0 / (2.0 / r - v2 / mu_sun))
        })?;
        let a = el.semi_major_axis;

        Ok(ThreatOrbitPreview {
            v_inf: enc.v_inf,
            impact_parameter: enc.impact_parameter,
            capture_radius: enc.capture_radius,
            is_hit: enc.impact_parameter <= enc.capture_radius,
            semi_major_axis_m: a,
            eccentricity: el.eccentricity,
            inclination_rad: el.inclination,
            period_seconds: std::f64::consts::TAU * (a * a * a / mu_sun).sqrt(),
        })
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
    if tier2.earth_j2 {
        // μ⊕ from the *same* ANISE `EARTH_J2000` GM the point-mass Earth uses, so
        // the oblateness correction and the monopole it corrects can never disagree
        // about Earth's mass — the rule the 1PN term follows for μ_sun. The J2/R_eq
        // pair travels together from the DE440 header (see `oblateness`).
        let mu_earth = eph.gm_km3_s2(EARTH_J2000)? * KM3_S2_TO_M3_S2;
        let earth = EphemerisPerturber::new(Arc::clone(eph), EARTH_J2000);
        let pole = EphemerisPole::earth(Arc::clone(eph));
        force = force.with(Box::new(Oblateness::earth_de440(mu_earth, earth, pole)));
    }
    if tier2.pluto {
        force = force.with(Box::new(pluto_perturber_field(eph)?));
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
    /// The designed impact state is not a hyperbolic flyby of Earth at all: the
    /// relative speed does not clear Earth escape at the offset distance, so
    /// there is no `v_inf` and no b-plane. Raised by
    /// [`ImpactorConfig::preview`] — the cheap wall in front of the expensive one.
    ImpactNotHyperbolic(String),
    /// A deflection evaluation/solve failed.
    Deflection(DeflectionError),
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScenarioError::KernelsNotFound(detail) => {
                write!(f, "no DE kernel pair could be resolved\n{detail}")
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
            ScenarioError::ImpactNotHyperbolic(m) => {
                write!(f, "designed impact is not a hyperbolic flyby: {m}")
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
        let force =
            compose_force(&eph, &cfg.tier2).map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
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
        let perp = impact_offset_axis(&vdir);
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
        self.with_toggled_field(tier2, |ds| Ok(ds.nominal_encounter()?))
    }

    /// Re-fly this scenario's built seed through a `tier2`-toggled field **and
    /// apply `delta_v` at `deflection_epoch`**, reporting the encounter the
    /// deflected pass reaches — the [`nominal_encounter_with`] measurement carried
    /// onto a *miss* geometry.
    ///
    /// Why this exists as a separate entry point: `nominal_encounter_with` can only
    /// ever measure a term on the shipping nominal, and that nominal is a designed
    /// **impact** whose closest approach is well inside Earth. For a term whose
    /// validity has a radial boundary — Earth's `J2`, whose expansion holds only
    /// outside `R_eq` — a number measured there is out of domain. A deflected pass
    /// is the geometry that actually matters (every successful deflection is one),
    /// and it is the only way to reach a wide perigee at all: [`build`](Self::build)
    /// verifies its designed impact round-trips, so a `b_offset_km` beyond the
    /// capture radius is rejected as "not a hit" rather than built as a miss.
    ///
    /// Same contract as the nominal sibling and for the same reason: the seed **and
    /// the impulse** are held fixed, and only the forward field changes, so the
    /// perigee difference between two calls is attributable to the term rather than
    /// to a re-planned deflection. Passing `&Tier2Config::default()` gives the
    /// baseline this scenario's own field would reach with that same impulse.
    ///
    /// `None` means the deflected pass left the scan's distance gate — a miss so
    /// wide there is no finite perigee to compare, which is a *successful*
    /// deflection but not a measurable one. Pick a smaller impulse.
    ///
    /// Cost: one full nominal propagation (the impulse is applied to the nominal
    /// state at `deflection_epoch`, so the nominal must be re-flown in this field
    /// too) plus the post-deflection arc — seconds, same as the sibling.
    ///
    /// [`nominal_encounter_with`]: Self::nominal_encounter_with
    pub fn deflected_encounter_with(
        &self,
        tier2: &Tier2Config,
        deflection_epoch: Epoch,
        delta_v: Vector3<f64>,
    ) -> Result<Option<BPlaneEncounter>, ScenarioError> {
        self.with_toggled_field(tier2, |ds| Ok(ds.evaluate(deflection_epoch, delta_v)?))
    }

    /// Build a [`DeflectionScenario`] over this scenario's **fixed seed** flown
    /// through a `tier2`-toggled field, and hand it to `f`.
    ///
    /// The single place a re-flown field is assembled, so the nominal and deflected
    /// measurement entry points above cannot drift apart in what "GR on" means or
    /// in which seed/cadence/scan they use — the same argument [`compose_force`]
    /// makes for the build and measurement paths. A closure rather than a returned
    /// value because the `DeflectionScenario` borrows the force built here.
    fn with_toggled_field<R>(
        &self,
        tier2: &Tier2Config,
        f: impl FnOnce(&DeflectionScenario<'_>) -> Result<R, ScenarioError>,
    ) -> Result<R, ScenarioError> {
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
        f(&ds)
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

    /// Reduce one initial state to b-plane geometry at a **fixed** epoch — the
    /// single sample the Tier-3 covariance mapping is built out of.
    ///
    /// Deliberately *not* "propagate, find this run's closest approach, reduce
    /// that". Closest approach is an argmin over a sampled polyline, so that map is
    /// quantised and its finite differences are noise; see the [`uncertainty`]
    /// module note. Every sample is reduced at the same `t_reduce` instead, which
    /// is legitimate because the b-plane parameters are asymptotic properties of
    /// the osculating geocentric hyperbola rather than of the sampling instant.
    ///
    /// [`uncertainty`]: crate::uncertainty
    fn uncertainty_sample(
        &self,
        seed: StateVector,
        t_reduce: Epoch,
        cadence_seconds: f64,
        n_snapshots: u32,
    ) -> Result<BPlaneEncounter, UncertaintyError> {
        let fail = |e: String| UncertaintyError::SampleFailed {
            column: None,
            message: e,
        };
        let clock = self
            .propagate_free(self.epoch0, seed, cadence_seconds, n_snapshots)
            .map_err(|e| fail(e.to_string()))?;
        let state = clock.state_at(t_reduce).map_err(|e| fail(e.to_string()))?;
        let earth = self
            .earth
            .state_at(t_reduce)
            .map_err(|e| fail(e.to_string()))?;
        BPlaneEncounter::from_relative_state(
            state.position - earth.position,
            state.velocity - earth.velocity,
            self.mu_earth,
            self.earth_radius,
        )
        .map_err(|e| fail(e.to_string()))
    }

    /// The fixed reduction epoch and the propagation shape every sample shares.
    ///
    /// Reduction happens [`UNCERTAINTY_REDUCTION_LEAD_SECONDS`] before the nominal
    /// closest approach, which for this campaign's ~7.6 km/s `v_inf` puts the rock
    /// about 330 000 km out: comfortably inside Earth's sphere of influence (so the
    /// osculating hyperbola is the encounter), comfortably outside the well (so
    /// `v_inf = √(v² − 2μ/r)` is not a catastrophic cancellation), and inside the
    /// scan gate. The propagation runs only to there plus one cadence of margin —
    /// there is no reason to fly past the epoch the answer is read at.
    ///
    /// **The anchor is the *first* encounter, indexed explicitly, and a second one
    /// is an error rather than a silent re-anchoring.** This used to read
    /// [`DeflectionScenario::nominal_encounter_epoch`], which reduces at the
    /// *minimum-distance* approach. Today the shipping span holds exactly one
    /// approach inside the gate, so the two agree and nothing was wrong. But the
    /// keyhole work extends the span past a resonant return that is *deeper* than
    /// the first encounter by construction — that is what a keyhole is — and on the
    /// day it does, a min-distance anchor relocates `t_reduce` from 12 h before
    /// encounter 1 to 12 h before encounter 2. Every column of the Jacobian would
    /// then describe a different encounter than the caller asked about, and none of
    /// it would error: the matrix stays finite, symmetric and plausible.
    ///
    /// So the fix is not to pick more cleverly. With two encounters in span, *which
    /// one the covariance is being mapped to* is a question only the caller can
    /// answer, and a chained two-encounter Jacobian is not defined here yet. Until
    /// it is, this refuses rather than guesses. `nominal_encounter_epoch` keeps its
    /// min-distance meaning for its ~30 other callers, who genuinely do want the
    /// closest pass.
    fn uncertainty_sampling_plan(&self) -> Result<(Epoch, f64, u32), UncertaintyError> {
        let fail = |e: String| UncertaintyError::SampleFailed {
            column: None,
            message: e,
        };
        let ds = self.deflection().map_err(|e| fail(e.to_string()))?;
        let approaches = find_close_approaches(ds.nominal(), &self.earth, self.scan)
            .map_err(|e| fail(e.to_string()))?;
        let first = approaches
            .first()
            .ok_or_else(|| fail("no nominal close approach inside the scan gate".into()))?;
        if approaches.len() > 1 {
            return Err(fail(format!(
                "the nominal span holds {} close approaches inside the scan gate, and a \
                 chained-encounter b-plane Jacobian is not defined here. Reduce the span to a \
                 single encounter, or extend the Tier-3 map to name which encounter it maps to. \
                 (Refusing rather than anchoring to one silently: the encounters are at {}.)",
                approaches.len(),
                approaches
                    .iter()
                    .map(|c| format!("{} ({:.0} km)", c.epoch.as_hifitime(), c.distance / 1.0e3))
                    .collect::<Vec<_>>()
                    .join(", "),
            )));
        }
        let t_reduce = first
            .epoch
            .shifted_by_seconds(-UNCERTAINTY_REDUCTION_LEAD_SECONDS);

        let cadence = SAMPLE_CADENCE_DAYS * 86_400.0;
        let span = t_reduce.tdb_seconds_past_j2000() - self.epoch0.tdb_seconds_past_j2000();
        if !(span.is_finite() && span > 0.0) {
            return Err(fail(format!(
                "reduction epoch is not after the campaign start (span {span} s)"
            )));
        }
        let n_snapshots = ((span / cadence).ceil() + 1.0) as u32;
        Ok((t_reduce, cadence, n_snapshots))
    }

    /// Map a state covariance at the campaign start onto the b-plane, and with it
    /// the impact probability (HANDOFF §7 Tier 3).
    ///
    /// This is the deterministic simulation's honest answer: not "does this rock
    /// hit" but "given what is actually known about where it is, how much of that
    /// spread lands on Earth". The result carries the 2×2 crossing covariance, the
    /// ellipse it implies, and [`BPlaneUncertainty::impact_probability`].
    ///
    /// **Cost: 13 propagations** — one nominal plus a central-difference pair per
    /// state component — at the [`SAMPLE_CADENCE_DAYS`] cadence, about 14 s for the
    /// shipping campaign. A measurement entry point, not something for a render
    /// loop.
    ///
    /// The covariance must describe the state at [`epoch0`](Self::epoch0), in
    /// barycentric ICRF metres and m/s, ordered `[r, v]` — the same frame and
    /// ordering the seed is in. A covariance from anywhere else silently answers a
    /// different question.
    /// Reuse [`bplane_sensitivity`](Self::bplane_sensitivity) when mapping more
    /// than one covariance — the 13 propagations are the covariance-independent
    /// part, and paying them per covariance turns a free comparison into a costly
    /// one.
    pub fn bplane_uncertainty(
        &self,
        covariance: &StateCovariance,
    ) -> Result<BPlaneUncertainty, UncertaintyError> {
        Ok(self.bplane_sensitivity()?.map(covariance))
    }

    /// [`bplane_uncertainty`](Self::bplane_uncertainty), plus the `±n σ` shell that
    /// says whether the linear map it rests on is still describing the encounter
    /// out at the edge of the covariance.
    ///
    /// `Σ_b = J Σ Jᵀ` is exact only for a linear map, and the state→b-plane map is
    /// not one. Whether the linearisation survives to 3σ is not something the
    /// Jacobian can report about itself, and it is not a question random sampling
    /// answers well — most draws land near the middle, where linearity was never in
    /// doubt. The principal-axis extremes are where it bends first and there are
    /// twelve of them.
    ///
    /// **Cost: 25 propagations** (~28 s) — the 13 above plus the shell.
    pub fn bplane_uncertainty_checked(
        &self,
        covariance: &StateCovariance,
        n_sigma: f64,
    ) -> Result<(BPlaneUncertainty, LinearityReport), UncertaintyError> {
        // One plan, shared: the shell must fly at the epoch the mean was measured
        // at, or the report is comparing reduction epochs and calling it curvature.
        let (sens, (t_reduce, cadence, n_snapshots)) = self.sensitivity_with_plan()?;
        let mean = sens.mean();

        let offsets = covariance.sigma_shell(n_sigma);
        let mut flown = Vec::with_capacity(offsets.len());
        for (i, o) in offsets.iter().enumerate() {
            let s = StateVector::new(
                self.seed.position + Vector3::new(o[0], o[1], o[2]),
                self.seed.velocity + Vector3::new(o[3], o[4], o[5]),
            );
            let enc = self
                .uncertainty_sample(s, t_reduce, cadence, n_snapshots)
                .map_err(|e| match e {
                    UncertaintyError::SampleFailed { message, .. } => {
                        UncertaintyError::SampleFailed {
                            column: Some(i),
                            message: format!("σ-shell sample: {message}"),
                        }
                    }
                    other => other,
                })?;
            flown.push(sens.basis.project(&enc) - mean);
        }

        let report = LinearityReport::new(&sens.jacobian, &offsets, &flown, n_sigma);
        Ok((sens.map(covariance), report))
    }

    /// The nominal fixed-epoch reduction, the frame it defines, and the 2×6
    /// Jacobian about it — the covariance-independent half of every Tier-3 answer.
    ///
    /// **Cost: 13 propagations** (~14 s). Hold onto the result: mapping any number
    /// of covariances through it afterwards is free
    /// ([`BPlaneSensitivity::map`](crate::uncertainty::BPlaneSensitivity::map)),
    /// which is what makes "how does the probability move as the orbit becomes
    /// better known" a question worth asking interactively.
    ///
    /// The nominal encounter it carries is the **fixed-epoch** reduction, not the
    /// closest-approach one [`nominal_hit`](Self::nominal_hit) gives. They agree to
    /// the asymptotic invariance of the hyperbola, but they are not the same
    /// number, and the mean of a distribution has to be measured by the same
    /// instrument as its spread or the two do not belong to one another.
    pub fn bplane_sensitivity(&self) -> Result<BPlaneSensitivity, UncertaintyError> {
        Ok(self.sensitivity_with_plan()?.0)
    }

    /// [`bplane_sensitivity`](Self::bplane_sensitivity), handing back the sampling
    /// plan it used rather than leaving a caller to re-derive one.
    ///
    /// The re-derivation is the hazard. A second
    /// [`uncertainty_sampling_plan`](Self::uncertainty_sampling_plan) call scans the
    /// nominal again and *today* returns the same `t_reduce`, so nothing is wrong —
    /// but nothing enforces it either, and the σ-shell differences its flown
    /// displacements against a mean measured at the **first** plan's epoch. Let the
    /// two drift apart and the linearity report compares two different reduction
    /// epochs and calls the difference nonlinearity: the module's own founding
    /// failure mode, one level up, wearing a plausible number. Threading the plan
    /// through makes "the mean and the spread were measured by the same instrument"
    /// a fact about the code rather than a claim in a doc comment.
    fn sensitivity_with_plan(
        &self,
    ) -> Result<(BPlaneSensitivity, (Epoch, f64, u32)), UncertaintyError> {
        let plan = self.uncertainty_sampling_plan()?;
        let (t_reduce, cadence, n_snapshots) = plan;
        let nominal = self.uncertainty_sample(self.seed, t_reduce, cadence, n_snapshots)?;
        let basis = BPlaneBasis::from_encounter(&nominal);
        let jacobian = bplane_jacobian(self.seed, |s| -> Result<Vector2<f64>, UncertaintyError> {
            let enc = self.uncertainty_sample(s, t_reduce, cadence, n_snapshots)?;
            Ok(basis.project(&enc))
        })?;
        Ok((
            BPlaneSensitivity {
                nominal,
                basis,
                jacobian,
            },
            plan,
        ))
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

    /// The loaded almanac for a kernel-gated test, or `None` to skip. Loading it
    /// directly (rather than through `RealFieldScenario::build`) is the whole point
    /// for the preview tests: they must be able to ask about a config *without*
    /// paying to build it.
    fn test_ephemeris(who: &str) -> Option<Arc<Ephemeris>> {
        let k = crate::kernels::resolve_for_test(who)?;
        let eph = Ephemeris::load(&k.bsp)
            .expect("DE kernel loads")
            .with_constants(&k.pca)
            .expect("planetary constants load");
        Some(Arc::new(eph))
    }

    /// Off-default threat orbits that must all build — the ones the frontend's
    /// knobs can actually reach. Named so a failure says which geometry broke.
    fn off_default_configs() -> Vec<(&'static str, ImpactorConfig)> {
        vec![
            (
                "faster (22 km/s)",
                ImpactorConfig {
                    v_rel_kms: 22.0,
                    ..Default::default()
                },
            ),
            (
                "steeper approach",
                ImpactorConfig {
                    v_rel_dir: Vector3::new(0.30, -0.90, -0.30),
                    ..Default::default()
                },
            ),
            (
                "wider offset (4200 km)",
                ImpactorConfig {
                    b_offset_km: 4_200.0,
                    ..Default::default()
                },
            ),
            // The long-period end, and the case most likely to break a patched-conic
            // estimate: catching Earth from behind puts `v_inf` *along* Earth's
            // 29.8 km/s, which is where `a` is most sensitive to the velocity the
            // preview reconstructs. If the preview holds here it holds anywhere the
            // direction knob can reach.
            (
                "near-prograde (long period)",
                ImpactorConfig {
                    v_rel_dir: Vector3::new(-0.95, -0.20, 0.05),
                    ..Default::default()
                },
            ),
        ]
    }

    /// **The preview's accuracy is a measurement, not a hope.**
    ///
    /// [`ImpactorConfig::preview`] reduces the designed impact in closed form at the
    /// impact epoch; [`RealFieldScenario::build_with`] reports vis-viva at the seed,
    /// `lead_years` (12) earlier, after a real integration through the perturbed
    /// field. Those are different quantities, and the only honest way to know
    /// whether the cheap one may be *labelled* with the expensive one's name is to
    /// difference them on configurations the UI can reach.
    ///
    /// The bound below is what the frontend's copy is allowed to claim. It is
    /// deliberately **not** tight enough to license substituting the preview for
    /// `period_seconds()` in anything that scores a plan — the tractor bench divides
    /// the lead by the period, and a few percent there walks straight into the
    /// margin.
    ///
    /// Kernel-gated; skips (does not fail) with no kernel. ~40 s: four builds.
    #[test]
    fn preview_tracks_the_built_orbit() {
        let Some(eph) = test_ephemeris("preview_tracks_the_built_orbit") else {
            return;
        };

        let mut worst_period = 0.0_f64;
        let mut cases = vec![("default", ImpactorConfig::default())];
        cases.extend(off_default_configs());

        for (who, cfg) in cases {
            let p = cfg.preview(&eph).expect("preview succeeds");
            let built = RealFieldScenario::build_with(&cfg, Arc::clone(&eph))
                .unwrap_or_else(|e| panic!("{who} must build: {e}"));

            let d_a =
                (p.semi_major_axis_m - built.semi_major_axis_m).abs() / built.semi_major_axis_m;
            let d_t = (p.period_seconds - built.period_seconds).abs() / built.period_seconds;
            worst_period = worst_period.max(d_t);

            // The designed geometry must also survive the round trip: the propagated
            // nominal's own b-plane reduction should land on the numbers the closed
            // form predicted from the impact state it was designed from.
            let enc = built
                .deflection()
                .expect("deflection scenario")
                .nominal_encounter()
                .expect("nominal encounter scans")
                .expect("nominal is a hit");
            let d_b = (p.impact_parameter - enc.impact_parameter).abs() / enc.impact_parameter;
            let d_v = (p.v_inf - enc.v_inf).abs() / enc.v_inf;

            eprintln!(
                "{who:24} T {:.4} yr (built {:.4}, {:+.3}%)  a {:+.3}%  \
                 b {:.0} km (built {:.0}, {:+.3}%)  v_inf {:+.3}%",
                p.period_seconds / SECONDS_PER_YEAR,
                built.period_seconds / SECONDS_PER_YEAR,
                100.0 * (p.period_seconds - built.period_seconds) / built.period_seconds,
                100.0 * (p.semi_major_axis_m - built.semi_major_axis_m) / built.semi_major_axis_m,
                p.impact_parameter / 1000.0,
                enc.impact_parameter / 1000.0,
                100.0 * (p.impact_parameter - enc.impact_parameter) / enc.impact_parameter,
                100.0 * (p.v_inf - enc.v_inf) / enc.v_inf,
            );

            // The encounter reduction is the same arithmetic on both sides, so it
            // must agree far more tightly than the 12-year orbit does. A loose bound
            // here would let a genuine geometry fork hide behind the orbit's slack.
            assert!(
                d_b < 1.0e-3 && d_v < 1.0e-3,
                "{who}: preview and built encounter geometry disagree \
                 (b {:.2e}, v_inf {:.2e}) — the two are not describing the same impact",
                d_b,
                d_v
            );
            assert!(
                d_a < 0.02 && d_t < 0.02,
                "{who}: preview orbit drifted from the built one (a {:.2e}, T {:.2e}) — \
                 re-measure before the UI keeps quoting it",
                d_a,
                d_t
            );
        }

        // Pinned so a regression that *worsens* the preview is visible even though
        // every individual case still passes its bound. Measured worst is 0.23%
        // (the steep approach); 1% leaves room for kernel/tolerance drift without
        // leaving room for the preview to quietly become a different orbit.
        assert!(
            worst_period < 0.01,
            "worst period error {:.3}% — update the doc and the UI's wording",
            100.0 * worst_period
        );
    }

    /// **Both walls are reachable from the knobs, and the cheap check finds them
    /// before the expensive one does.**
    ///
    /// The two are not the same wall and they close from opposite directions:
    ///
    /// - *Too slow for the offset.* `r_rel ⊥ v_rel` puts the impact at the perigee
    ///   of the geocentric hyperbola, so the flyby exists only while
    ///   `v_rel > √(2μ⊕/b_offset)` — 16.3 km/s at the shipping 3 000 km offset,
    ///   against a shipping `v_rel` of 18. **Shrinking the offset raises that bar**
    ///   (28.2 km/s at 1 000 km), so pulling the hit toward Earth's centre is what
    ///   falls off the cliff, which is the opposite of the intuition.
    /// - *Too wide to be a hit.* The asymptote misses by `b = b_offset·v_rel/v_inf`,
    ///   **not** by `b_offset` — 7 077 km for the shipping 3 000 km, against an
    ///   11 311 km capture disc. But `b` and `b_capture` do **not** race each
    ///   other freely, and the naive "b hits 11 311 km at ~4 800 km of offset" is
    ///   wrong: widening the offset also *raises* `v_inf` (less of Earth's well to
    ///   climb out of), which grows `b` more slowly and shrinks `b_capture` at the
    ///   same time. The two meet at a value that is not a coincidence —
    ///   `b ≤ b_capture ⟺ r_perigee ≤ R⊕`, and with `r_rel ⊥ v_rel` the perigee
    ///   **is** `b_offset`. So the offset knob's ceiling is **exactly Earth's
    ///   radius**, and `b_offset` is really a perigee-altitude dial wearing a
    ///   b-plane name.
    ///
    /// The second assertion is the one that earns the preview its keep: the same
    /// geometry is handed to `build_with`, which rejects it — after a 10 s
    /// back-propagation the preview refused in microseconds.
    ///
    /// Kernel-gated; skips (does not fail) with no kernel.
    #[test]
    fn preview_finds_both_walls_before_the_builder_pays_for_them() {
        let Some(eph) = test_ephemeris("preview_finds_both_walls_*") else {
            return;
        };

        // --- The shipping config sits between the walls, and b ≠ b_offset --------
        let cfg = ImpactorConfig::default();
        let base = cfg.preview(&eph).expect("default previews");
        assert!(
            base.is_hit,
            "the shipping config must still be a designed hit"
        );
        assert!(
            base.impact_parameter > 2.0 * cfg.b_offset_km * KM_TO_M,
            "focusing must widen the asymptote's miss well past the aim point: \
             b = {:.0} km for a {:.0} km offset",
            base.impact_parameter / 1000.0,
            cfg.b_offset_km,
        );
        assert!(
            base.impact_parameter < base.capture_radius,
            "b {:.0} km must sit inside the capture disc {:.0} km",
            base.impact_parameter / 1000.0,
            base.capture_radius / 1000.0,
        );

        // --- Wall 1: too slow for the offset -------------------------------------
        // Same 18 km/s that works at 3 000 km, at an offset where escape is 28 km/s.
        let slow = ImpactorConfig {
            b_offset_km: 1_000.0,
            ..Default::default()
        };
        match slow.preview(&eph) {
            Err(ScenarioError::ImpactNotHyperbolic(m)) => {
                assert!(m.contains("escape"), "the message must name the bar: {m}")
            }
            other => panic!("a 1 000 km offset at 18 km/s is not a flyby, got {other:?}"),
        }

        // --- Wall 2: too wide to be a hit ----------------------------------------
        // Walk the offset out until the preview says it stopped being an impact,
        // then hold `build_with` to the same verdict.
        let mut first_miss = None;
        for offset_km in (3_000..=9_000).step_by(50) {
            let c = ImpactorConfig {
                b_offset_km: offset_km as f64,
                ..Default::default()
            };
            if !c.preview(&eph).expect("previews").is_hit {
                first_miss = Some(c);
                break;
            }
        }
        let missing = first_miss.expect("the offset knob must be able to leave the capture disc");
        let r_earth_km = geometry::EARTH_EQUATORIAL_RADIUS_M / 1000.0;
        eprintln!(
            "preview stops calling it a hit at b_offset = {:.0} km (R_earth = {:.0} km)",
            missing.b_offset_km, r_earth_km,
        );
        // **The wall is Earth's radius, exactly** — see the doc above. Pinned on the
        // *derived* value: a bound fitted to whatever the sweep printed would ratify
        // a focusing bug, since a wrong `v_inf` moves `b` and `b_capture` together
        // and `is_hit` would keep flipping somewhere plausible-looking.
        assert!(
            (missing.b_offset_km - r_earth_km).abs() <= 50.0,
            "the offset wall must land on Earth's radius {r_earth_km:.0} km \
             (the impact point is the hyperbola's perigee), found {:.0} km",
            missing.b_offset_km,
        );
        // The expensive confirmation. This is the whole justification for the
        // preview: the builder agrees, and charges 10 s to say so.
        match RealFieldScenario::build_with(&missing, Arc::clone(&eph)) {
            Err(ScenarioError::NominalNotAHit(_)) => {}
            other => panic!(
                "build_with must reject the geometry the preview refused, got {:?}",
                other.map(|_| "a built scenario")
            ),
        }
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
        if crate::kernels::resolve_for_test("propagate_free_matches_direct_step_in_the_field")
            .is_none()
        {
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
        if crate::kernels::resolve_for_test("encounter_frame_track_agrees_with_reported_perigee")
            .is_none()
        {
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
    /// The Tier-3 reduction epoch anchors to the **first** encounter in span, and
    /// today that is the same as the minimum-distance one because there is only one.
    ///
    /// This pins the equivalence rather than assuming it, and it is deliberately a
    /// tripwire: the day someone extends the span past a resonant return — which is
    /// the whole point of the keyhole work, and which lands *deeper* than encounter
    /// 1 by construction — the census stops returning a single approach and
    /// `uncertainty_sampling_plan` refuses. That refusal is the designed behaviour,
    /// so this test failing means "go decide which encounter the map is about," not
    /// "the plan broke."
    ///
    /// Without this, the anchor would have silently relocated to encounter 2 and
    /// every Jacobian column would have described a different encounter than the
    /// caller asked about, with nothing erroring — the matrix stays finite,
    /// symmetric, and entirely plausible.
    #[test]
    fn the_tier3_reduction_epoch_anchors_to_the_first_encounter_and_refuses_a_second() {
        if crate::kernels::resolve_for_test("tier3_reduction_epoch_anchors_to_the_first").is_none() {
            return;
        }

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");
        let ds = sc.deflection().expect("deflection");

        let approaches = find_close_approaches(ds.nominal(), &sc.earth, sc.scan)
            .expect("census the nominal span");
        assert_eq!(
            approaches.len(),
            1,
            "the shipping span is supposed to hold exactly one encounter inside the gate; \
             found {} at {:?}. If this is intentional, uncertainty_sampling_plan now refuses \
             and the Tier-3 map needs to name its encounter.",
            approaches.len(),
            approaches
                .iter()
                .map(|c| (c.epoch.as_hifitime().to_string(), c.distance / 1.0e3))
                .collect::<Vec<_>>(),
        );

        // First and minimum-distance agree today, which is why the change of anchor
        // is behaviour-preserving. Asserting it means a future divergence surfaces
        // here rather than inside a Jacobian column.
        let closest = ds
            .nominal_encounter_epoch()
            .expect("encounter epoch")
            .expect("an encounter");
        assert_eq!(
            approaches[0].epoch.tdb_seconds_past_j2000(),
            closest.tdb_seconds_past_j2000(),
            "first-in-span and minimum-distance must name the same encounter while there is \
             only one"
        );

        // And the plan really is 12 h before it.
        let (t_reduce, _cadence, _n) = sc
            .uncertainty_sampling_plan()
            .expect("the single-encounter span yields a plan");
        let lead = approaches[0].epoch.tdb_seconds_past_j2000() - t_reduce.tdb_seconds_past_j2000();
        assert!(
            (lead - UNCERTAINTY_REDUCTION_LEAD_SECONDS).abs() < 1.0e-3,
            "reduction lead is {lead} s, expected {UNCERTAINTY_REDUCTION_LEAD_SECONDS} s"
        );
    }

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
        let Some(k) = crate::kernels::resolve_for_test("asteroid_perturbers_…_shift_it_on")
        else {
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

    /// The last two Tier-2 terms — Earth's `J2` and Pluto — measured the same
    /// fixed-seed way, and the measurement that answers the open question each of
    /// them was parked on.
    ///
    /// **`J2`.** Deferred through the whole of Tier 2 as "negligible
    /// heliocentrically", which is true and is exactly why it must be measured at
    /// the *encounter* rather than argued about: the term is `1/r⁴`, so essentially
    /// all of its effect is bought in the minutes the asteroid spends inside a few
    /// Earth radii. Whatever comes out is the honest size of that, reported.
    ///
    /// **Read the `J2` line with its caveat.** This scenario's nominal is a designed
    /// *impact*, so its closest approach (3000 km) is **inside** `R_eq`, where the
    /// `J2` expansion is outside its domain of validity. The print therefore also
    /// reports the **capture radius**, which moves 11 311.3 → 11 389.0 km with `J2`
    /// on: the b-plane reduction infers `v_∞` from the point-mass energy at that
    /// sub-surface sample point, and `J2`'s potential correction there is large
    /// enough to bias it ~1%. 1PN, whose correction at that radius is ~1e-9
    /// relative, leaves the capture radius unchanged to the digit — the control that
    /// says this is `J2`'s `1/r⁴` growth inside the body, not the reduction. For any
    /// *miss* geometry (perigee outside `R_eq`) the term is in its valid domain and
    /// this does not arise. See `forces::oblateness`.
    ///
    /// **Pluto.** HANDOFF parked "Pluto in the shipping field" on a missing GM,
    /// which the DE440 header supplies, and on an unmeasured cost — batch-2c's
    /// ~55 m over two years for a main-belt particle, *growing with lead time*.
    /// This runs it at the campaign's real ~12 yr lead so the 10-vs-11-body choice
    /// is made against a number instead of an extrapolation.
    ///
    /// Assertions stay structural (off == baseline bit-for-bit; on shifts by a
    /// nonzero finite amount); the magnitudes are printed, never asserted, because
    /// both terms are expected small and manufacturing a threshold they must clear
    /// is how a measurement turns into a claim.
    #[test]
    fn earth_j2_and_pluto_leave_the_bplane_unchanged_off_and_shift_it_on() {
        if crate::kernels::resolve_for_test("earth_j2_and_pluto_…_shift_it_on").is_none() {
            return;
        }

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");
        let baseline = sc
            .deflection()
            .expect("deflection")
            .nominal_encounter()
            .expect("nominal reduces")
            .expect("nominal is a hit");

        // (a) Both off == the shipping baseline, bit-for-bit.
        let off = sc
            .nominal_encounter_with(&Tier2Config::default())
            .expect("off re-fly")
            .expect("off pass is still an encounter");
        assert_eq!(
            off.perigee, baseline.perigee,
            "all-off re-fly must match the shipping perigee bit-for-bit"
        );

        // (b) Earth's J2 — bought almost entirely during the final close pass.
        let j2 = sc
            .nominal_encounter_with(&Tier2Config {
                earth_j2: true,
                ..Tier2Config::default()
            })
            .expect("J2 re-fly")
            .expect("J2 pass is still an encounter");
        let j2_shift = (j2.perigee - baseline.perigee).abs();
        println!(
            "Earth J2 (DE440 J2E, real spin axis): perigee shift over the campaign {:.4} km \
             (baseline {:.1} km → +J2 {:.1} km); capture radius {:.1} → {:.1} km",
            j2_shift / 1e3,
            baseline.perigee / 1e3,
            j2.perigee / 1e3,
            baseline.capture_radius / 1e3,
            j2.capture_radius / 1e3,
        );
        assert!(
            j2_shift > 0.0 && j2_shift.is_finite(),
            "Earth's J2 should move the perigee by a nonzero finite amount, got {j2_shift:.3e} m"
        );

        // (c) Pluto — ASSIST's 11th point mass, the one the shipping ten omit.
        let pluto = sc
            .nominal_encounter_with(&Tier2Config {
                pluto: true,
                ..Tier2Config::default()
            })
            .expect("Pluto re-fly")
            .expect("Pluto pass is still an encounter");
        let pluto_shift = (pluto.perigee - baseline.perigee).abs();
        println!(
            "Pluto (DE440 GM9, 11th point mass): perigee shift over the campaign {:.4} km \
             (baseline {:.1} km → +Pluto {:.1} km)",
            pluto_shift / 1e3,
            baseline.perigee / 1e3,
            pluto.perigee / 1e3,
        );
        assert!(
            pluto_shift > 0.0 && pluto_shift.is_finite(),
            "Pluto should move the perigee by a nonzero finite amount, got {pluto_shift:.3e} m"
        );
    }

    /// Lead time of the miss-geometry impulse before impact, seconds — one year.
    ///
    /// Deliberately *not* the campaign's full ~12 yr lead, and the reason is cost,
    /// not physics: the solve that picked this geometry
    /// (`examples/probe_miss_geometry.rs`) re-propagates from the impulse to the
    /// span end on every bisection step, so an impulse at the campaign start makes
    /// each of ~30 steps a full 12 yr flight. What is being fixed here is a
    /// *perigee*; the lead time only sets how much Δv buys it.
    const MISS_LEAD_SECONDS: f64 = 365.25 * 86_400.0;
    /// Along-track impulse magnitude, m/s, solved by that probe to put the deflected
    /// perigee at 3.0 `R_eq`. Hardcoded rather than re-solved so the geometry is
    /// fixed by construction and the test costs three propagations, not thirty.
    const MISS_DV_M_S: f64 = 0.399_625;

    /// Measure Earth's `J2` on a geometry where it is **valid** — and show that the
    /// capture-radius anomaly the impact geometry produces is bought inside `R_eq`.
    ///
    /// The sibling test above measures `J2` on the shipping nominal and records
    /// 1.33 km. That number grazes a validity boundary: the nominal is a designed
    /// impact with closest approach 3000 km, *inside* Earth, while the `J2`
    /// expansion holds only outside `R_eq`. Its visible symptom is the b-plane
    /// reduction, which infers `v_∞` from **point-mass** energy at the sampled
    /// closest approach and so picks up `J2`'s potential correction there — moving
    /// the capture radius by 0.69 % against a perigee shift of 1.33 km.
    ///
    /// The claim in the module docs is *causal*: that anomaly is `J2` evaluated deep
    /// inside the body, not a defect in the reduction. So the assertion that would
    /// **fail if the explanation were wrong** is not "the shift is nonzero" (true
    /// anywhere) but that the anomaly *collapses with distance*: the correction goes
    /// as `(μ/r)·J2·(R_eq/r)²`, i.e. `1/r³`, so at a perigee 6.4× wider it must fall
    /// by ~260×. A reduction that were simply biased would not care about `r`.
    ///
    /// The geometry is a deflected pass, which is both the only way to reach a wide
    /// perigee ([`RealFieldScenario::build`] rejects a designed miss as "not a hit")
    /// and the case that actually matters, since every successful deflection is one.
    #[test]
    fn earth_j2_on_a_deflected_miss_is_in_domain() {
        if crate::kernels::resolve_for_test("earth_j2_on_a_deflected_miss_is_in_domain").is_none() {
            return;
        }

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");
        let ds = sc.deflection().expect("deflection");
        let nominal = ds
            .nominal_encounter()
            .expect("nominal reduces")
            .expect("nominal is a hit");

        let t_d = sc.impact_epoch().shifted_by_seconds(-MISS_LEAD_SECONDS);
        let dir = crate::deflection::along_track_unit(
            ds.nominal().state_at(t_d).expect("nominal state at t_d"),
        )
        .expect("along-track direction");
        let dv = MISS_DV_M_S * dir;

        // (a) The new entry point, terms off, reproduces the scenario's own deflected
        // pass bit-for-bit — the same "unchanged with them off" identity the nominal
        // sibling relies on, and what makes a difference between two of its calls
        // attributable to the term rather than to a second code path.
        let direct = ds
            .evaluate(t_d, dv)
            .expect("deflected pass")
            .expect("deflected pass is still an encounter");
        let base = sc
            .deflected_encounter_with(&Tier2Config::default(), t_d, dv)
            .expect("all-off deflected re-fly")
            .expect("all-off deflected pass is still an encounter");
        assert_eq!(
            base.perigee, direct.perigee,
            "all-off deflected re-fly must match the scenario's own deflected perigee bit-for-bit"
        );

        // (b) The preconditions, asserted rather than assumed: this pass really is a
        // miss, and its perigee really is outside Earth. Without both, the
        // measurement below is just the sibling test in a longer costume.
        assert!(
            base.impact_parameter > base.capture_radius,
            "the chosen impulse must open a clean miss: |B| {:.1} km vs capture {:.1} km",
            base.impact_parameter / 1e3,
            base.capture_radius / 1e3,
        );
        assert!(
            base.perigee > base.earth_radius,
            "the miss geometry must sit outside R_eq for J2 to be in domain: perigee {:.1} km \
             vs R_eq {:.1} km",
            base.perigee / 1e3,
            base.earth_radius / 1e3,
        );

        // (c) J2 on that miss — the in-domain number, signed as the frontend signs
        // every shift (positive = pulled inward).
        let j2_miss = sc
            .deflected_encounter_with(
                &Tier2Config {
                    earth_j2: true,
                    ..Tier2Config::default()
                },
                t_d,
                dv,
            )
            .expect("J2 deflected re-fly")
            .expect("J2 deflected pass is still an encounter");
        let miss_shift_km = (base.perigee - j2_miss.perigee) / 1e3;

        // (d) The same term on the impact geometry, measured here rather than quoted,
        // so the comparison below is between two numbers from one run.
        let j2_hit = sc
            .nominal_encounter_with(&Tier2Config {
                earth_j2: true,
                ..Tier2Config::default()
            })
            .expect("J2 nominal re-fly")
            .expect("J2 nominal pass is still an encounter");
        let hit_shift_km = (nominal.perigee - j2_hit.perigee) / 1e3;

        let rel_hit =
            (j2_hit.capture_radius - nominal.capture_radius).abs() / nominal.capture_radius;
        let rel_miss = (j2_miss.capture_radius - base.capture_radius).abs() / base.capture_radius;

        // (e) The control that names the mechanism: 1PN on the *same* miss geometry.
        // Its correction at closest approach is ~1e-9 relative, so if the capture
        // radius moved for any reason other than the term's own potential reaching
        // into the reduction, it would move here too.
        let gr_miss = sc
            .deflected_encounter_with(
                &Tier2Config {
                    relativity: true,
                    ..Tier2Config::default()
                },
                t_d,
                dv,
            )
            .expect("1PN deflected re-fly")
            .expect("1PN deflected pass is still an encounter");
        let rel_gr = (gr_miss.capture_radius - base.capture_radius).abs() / base.capture_radius;

        println!(
            "Earth J2, two geometries, one scenario:\n  \
             IMPACT (nominal, out of domain): perigee {:.1} km = {:.3} R_eq, shift {:+.4} km, \
             capture {:.1} → {:.1} km ({:.4} %)\n  \
             MISS ({:.6} m/s along-track, {:.1} yr lead, IN domain): perigee {:.1} km = \
             {:.3} R_eq, shift {:+.4} km, capture {:.1} → {:.1} km ({:.5} %)\n  \
             1PN control on the same miss: capture {:.1} → {:.1} km ({:.2e} relative)",
            nominal.perigee / 1e3,
            nominal.perigee / nominal.earth_radius,
            hit_shift_km,
            nominal.capture_radius / 1e3,
            j2_hit.capture_radius / 1e3,
            rel_hit * 100.0,
            MISS_DV_M_S,
            MISS_LEAD_SECONDS / (365.25 * 86_400.0),
            base.perigee / 1e3,
            base.perigee / base.earth_radius,
            miss_shift_km,
            base.capture_radius / 1e3,
            j2_miss.capture_radius / 1e3,
            rel_miss * 100.0,
            base.capture_radius / 1e3,
            gr_miss.capture_radius / 1e3,
            rel_gr,
        );

        // The impact geometry is out of domain and says so.
        assert!(
            rel_hit > 1.0e-3,
            "the impact geometry should show the out-of-domain capture-radius bias \
             (expected ~0.69 %), got {:.3e} relative",
            rel_hit
        );
        // The collapse, two ways. First model-free: an order of magnitude at least.
        assert!(
            rel_miss < rel_hit / 10.0,
            "the capture-radius bias must collapse on a geometry outside R_eq: \
             miss {rel_miss:.3e} vs impact {rel_hit:.3e} relative"
        );
        // Then against the 1/r³ the mechanism predicts, with slack for the Legendre
        // factor, which depends on the latitude of closest approach and differs
        // between the two passes. Only an upper bound: P₂ can shrink this to nothing
        // but cannot inflate it past ~1.
        let predicted = rel_hit * (nominal.perigee / base.perigee).powi(3);
        assert!(
            rel_miss < 3.0 * predicted,
            "the bias should fall as (μ/r)·J2·(R_eq/r)² ∝ 1/r³: predicted ≲ {predicted:.3e}, \
             measured {rel_miss:.3e} relative"
        );
        // And the control: nothing about the reduction itself moves the capture radius.
        assert!(
            rel_gr < 1.0e-5,
            "1PN must leave the capture radius essentially untouched — a change here would \
             mean the bias is not J2's potential reaching the reduction, got {rel_gr:.3e}"
        );

        // The frontend cites this number in the force-menu footnote; pin it so the
        // caption cannot drift from the physics (the SB441_BODIES treatment).
        let recorded = super::J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM;
        assert!(
            (miss_shift_km - recorded).abs() <= 0.02 * recorded.abs().max(1.0e-3),
            "J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM is {recorded:+.4} km but the miss geometry \
             measures {miss_shift_km:+.4} km — update the constant (the frontend prints it)"
        );
    }
    use crate::uncertainty::StateCovariance;

    /// The Tier-3 pipeline, end to end against the real field.
    ///
    /// The kernel-free tests in [`crate::uncertainty`] validate the *mathematics* —
    /// the difference scheme against an exactly-linear map, the probability integral
    /// against the Rayleigh closed form. None of them says a 12-year arc reduces to
    /// a sane Jacobian, which is a separate claim and needs the real propagator.
    ///
    /// The discriminating assertion is the first one: the **fixed-epoch** reduction
    /// this module is built on must agree with the **closest-approach** reduction
    /// everything else in the crate uses. That is the design's founding assumption,
    /// it is the cheapest thing here to get invisibly wrong (a wrong reduction epoch
    /// still yields a full, plausible, entirely incorrect Jacobian), and nothing
    /// else in the suite would catch it. Measured at 0.025% by
    /// `probe_tier3_uncertainty`; the 2% gate here is loose enough not to be flaky
    /// and tight enough that a broken epoch — which would be off by far more than a
    /// percent — cannot pass.
    ///
    /// Kernel-gated; skips (does not fail) with no kernel. ~30 s.
    #[test]
    fn tier3_covariance_maps_to_the_bplane_on_the_real_field() {
        if crate::kernels::resolve_for_test("tier3_covariance_maps_to_the_bplane_*").is_none() {
            return;
        }
        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");
        let ds = sc.deflection().expect("deflection");
        let seed = ds.nominal().state_at(sc.epoch0()).expect("seed");
        let nominal_at_ca = sc.nominal_hit(&ds).expect("nominal hit");

        let sens = sc.bplane_sensitivity().expect("sensitivity");

        // The founding assumption: same encounter, two different reduction rules.
        let rel = (sens.nominal.perigee - nominal_at_ca.perigee).abs() / nominal_at_ca.perigee;
        assert!(
            rel < 0.02,
            "fixed-epoch reduction disagrees with the closest-approach reduction by {:.3}%              (perigee {:.1} km vs {:.1} km) — the b-plane parameters are supposed to be              asymptotic, so a gap this size means the reduction epoch is wrong",
            rel * 100.0,
            sens.nominal.perigee / 1e3,
            nominal_at_ca.perigee / 1e3
        );
        let rel_v = (sens.nominal.v_inf - nominal_at_ca.v_inf).abs() / nominal_at_ca.v_inf;
        assert!(rel_v < 0.02, "v_inf disagrees by {:.3}%", rel_v * 100.0);

        // …and the *derivative*, which is the property the module actually rests on
        // rather than a correlate of it — ∂r_p/∂v_along computed at the fixed epoch
        // and at each run's own closest approach, four propagations. Measured at
        // 0.025%; gated at 1%.
        //
        // Both gates were probed by moving the reduction lead and watching this test:
        // 12 d fails at 107%, 48 h at 6.5%, 30 h at 2.3%, and 26 h passes cleanly.
        // So the asymptotic invariance holds out to about a day and the shipping
        // 12 h has ~2.5× margin — and, honestly, on *this* scenario the perigee gate
        // is the one that trips first, so the derivative gate did not prove tighter.
        // It stays because it checks the claim directly instead of by proxy, and a
        // faster encounter or a different geometry need not preserve that ordering.
        let plan = sc.uncertainty_sampling_plan().expect("sampling plan");
        let (t_reduce, cadence, n_snap) = plan;
        let t_hat = crate::deflection::along_track_unit(seed).expect("along-track");
        let h = crate::uncertainty::FD_STEP_VELOCITY_MS;
        let mut fixed = [0.0_f64; 2];
        let mut at_ca = [0.0_f64; 2];
        for (i, sign) in [1.0_f64, -1.0].iter().enumerate() {
            let s = StateVector::new(seed.position, seed.velocity + sign * h * t_hat);
            fixed[i] = sc
                .uncertainty_sample(s, t_reduce, cadence, n_snap)
                .expect("fixed-epoch sample")
                .perigee;
            let clock = sc
                .propagate_free(sc.epoch0(), s, cadence, n_snap)
                .expect("propagate");
            at_ca[i] = crate::close_approach::closest_approach(&clock, &sc.earth, sc.scan)
                .expect("scan")
                .expect("close approach in gate")
                .b_plane(sc.mu_earth, sc.earth_radius)
                .expect("b-plane")
                .perigee;
        }
        let d_fixed = (fixed[0] - fixed[1]) / (2.0 * h);
        let d_at_ca = (at_ca[0] - at_ca[1]) / (2.0 * h);
        let d_rel = (d_fixed - d_at_ca).abs() / d_at_ca.abs();
        assert!(
            d_rel < 0.01,
            "∂r_p/∂v_along disagrees by {:.4}% between the fixed-epoch reduction \
             ({d_fixed:.5e}) and the closest-approach one ({d_at_ca:.5e}) — the Jacobian is \
             built on those being the same measurement",
            d_rel * 100.0
        );

        // The projected mean must carry the impact parameter — if it does not, the
        // frame is not the b-plane's and every covariance pushed through it is
        // expressed in the wrong plane.
        let mean = sens.mean();
        assert!(
            (mean.norm() - sens.nominal.impact_parameter).abs()
                < 1e-6 * sens.nominal.impact_parameter,
            "projected mean {:.3} km vs |B| {:.3} km",
            mean.norm() / 1e3,
            sens.nominal.impact_parameter / 1e3
        );

        // Every column finite and non-trivial: a silently-zero column would make the
        // covariance confidently wrong in exactly that direction.
        for c in 0..6 {
            let n = sens.jacobian.column(c).norm();
            assert!(n.is_finite() && n > 0.0, "jacobian column {c} is {n}");
        }
        // Velocity columns dominate — a velocity error has the whole campaign to
        // become a position error. Measured ~4e6 s of ratio; asserted loosely as
        // "at least a thousand-fold", which a units error could not survive.
        let pos = (0..3)
            .map(|c| sens.jacobian.column(c).norm())
            .fold(0.0_f64, f64::max);
        let vel = (3..6)
            .map(|c| sens.jacobian.column(c).norm())
            .fold(0.0_f64, f64::max);
        assert!(
            vel / pos > 1.0e3,
            "velocity/position column ratio {:.3e}",
            vel / pos
        );

        // An along-track-dominated covariance must map to an elongated ellipse. A
        // near-circular one would contradict the deflection curve's own finding that
        // along-track is the sensitive direction.
        let cov = StateCovariance::synthetic_along_track(seed, 5.0e-5, 20.0, 1.0e3)
            .expect("non-degenerate seed");
        let mapped = sens.map(&cov);
        let (major, minor) = mapped.sigma_axes();
        assert!(major > minor && minor > 0.0);
        assert!(
            major / minor > 10.0,
            "b-plane ellipse {:.1} km x {:.1} km is too round for a 20:1 along-track cigar",
            major / 1e3,
            minor / 1e3
        );

        // The nominal is a designed hit sitting well inside a capture disc far wider
        // than this ellipse, so the probability is 1 — and a pipeline that reported
        // anything else would be broken in a way the ellipse shape alone would hide.
        let p = mapped.impact_probability().expect("well-posed");
        assert!(
            p > 0.999,
            "designed hit with a sub-disc ellipse gave P = {p:.6e}"
        );

        // Widen the covariance until the spread reaches outside Earth and the
        // probability must fall strictly below 1 — the whole point of the layer.
        let wide = StateCovariance::synthetic_along_track(seed, 3.0e-2, 20.0, 1.0e3)
            .expect("non-degenerate seed");
        let p_wide = sens.map(&wide).impact_probability().expect("well-posed");
        assert!(
            p_wide > 0.01 && p_wide < 0.9,
            "a poorly-known orbit on the same hitting trajectory gave P = {p_wide:.6e};              expected a partial probability, since the ellipse now reaches past Earth"
        );
    }
}

#[cfg(test)]
mod _sync_gate {
    fn _assert_sync<T: Sync>() {}
    #[test]
    fn real_field_scenario_is_sync() {
        // The gate for the Arc-shared Tier-2 preview: the gdext binding clones an
        // Arc<RealFieldScenario> to a worker thread and measures shifts off it while
        // the render thread keeps reading the same scenario. That needs Sync.
        _assert_sync::<super::RealFieldScenario>();
    }
}
