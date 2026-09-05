//! Keyhole **targeting**: aim a deflection at a resonant-return circle, fly it
//! through the real field, and refine until the return is as close as the physics
//! allows — the generalisation of `probe_keyhole_return`'s one hardcoded shot.
//!
//! [`keyhole`](crate::keyhole) is a map: closed-form circles, gradients and
//! keyhole widths, computed in microseconds and honest about what they cannot
//! say. Its module doc states the boundary — absolute placement of `a'` carries
//! `δa'/a' ≈ 1.3e-4`, which over an `h`-year return is hours of arrival slip and
//! ~10⁶ km of Earth's motion against a keyhole tens of kilometres wide. **The map
//! aims; only a flown trajectory answers.** This module is the flying.
//!
//! # The three steps, and why each exists
//!
//! 1. **Aim** ([`aim_at_resonance`]). Pick the point of the resonant circle to
//!    fly through, convert it to the perigee a deflection solver targets
//!    (`b² = r_p² + 2c·r_p`), and solve the impulse magnitude that reaches it.
//!    Closed-form except the solve; the solve is the expensive part (~2–4 min on
//!    the shipping rock, because every bracket step re-flies the campaign).
//! 2. **Fly** ([`fly_keyhole_shot`]). Re-fly the deflection, reduce encounter 1,
//!    hand off well outside Earth's sphere of influence, propagate the resonant
//!    orbit forward and census Earth approaches inside a **wide** gate — wide
//!    because the closed form's timing error is far outside any shipping gate,
//!    so a narrow gate would report "no return" for a return that is there.
//! 3. **Refine** ([`solve_keyhole_return`]). The return miss is V-shaped in Δv —
//!    it is Earth's own motion over the arrival slip — so a golden-section search
//!    drives it to its floor. The floor is not zero and is not a convergence
//!    failure: it is the post-encounter orbit's spatial offset at the return,
//!    which no change of timing can remove.
//!
//! # The return is reduced in **its own** Öpik frame
//!
//! Encounter 1's frame is built from Earth's heliocentric state at encounter 1.
//! The return happens `h` years later, with Earth somewhere else on its orbit and
//! the asteroid arriving on a different asymptote, so encounter 1's `(ξ̂, ζ̂)` mean
//! nothing there. [`FlownReturn`] therefore carries a second frame, built at the
//! return epoch, and that is what makes the floor readable rather than merely
//! numeric:
//!
//! - **`ζ` is the timing coordinate** (`ζ̂` opposes Earth's motion), and Δv is a
//!   timing knob — it moves *when* the rock arrives, hardly at all *where* the
//!   orbits pass.
//! - **`ξ` is the spatial offset** — how far the two orbits miss in space, to
//!   first order the MOID.
//!
//! So a converged refinement must drive `ζ₂ → 0` and leave the floor in `|ξ₂|`.
//! A floor sitting mostly in `ζ₂` is a **search that has not converged wearing a
//! physics costume**, and the scalar return distance alone cannot tell the two
//! apart. That split is the check this module exists to make possible.
//!
//! **Measured on the flown 3:4 return (2026-09-05):** `ξ₂ = 4 014 km`,
//! `ζ₂ = 891 km` — 78 % spatial, so the search converged. Its impact parameter is
//! 4 112 km and its geocentric closest approach 1 141 km; the gap between those
//! two is gravitational focusing, and mixing them up is the same trap
//! `geometry.rs` documents for the first encounter.
//!
//! **Where the same discipline is deliberately *not* applied.** A planner readout
//! that places a *deflected* b-point on the *nominal* encounter's circles is
//! sound, and the difference is a scale argument rather than a principle: the
//! circles depend on the encounter only through `c = μ⊕/v∞²` and `θ`, and a small
//! along-track nudge years out moves where the rock arrives without much changing
//! how fast or from what direction it gets there. Measured on the shipping plan,
//! rebuilding the frame from the deflected encounter moves the 3:4 circle 1.5 km
//! in centre and 1.6 km in radius against a 24.9 km door. A return `h` years
//! later has no such excuse — it is a different encounter, with Earth elsewhere.
//!
//! # Cost
//!
//! One [`fly_keyhole_shot`] is a campaign re-fly plus an `h`-year propagation:
//! ~15 s on the shipping rock in release. A full [`solve_keyhole_return`] is the
//! aim solve plus ~15–20 flights: **~4 minutes**. Nothing here belongs on a build
//! path or a frame; it is an on-demand worker at best, and an example at least.

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use nalgebra::{Vector2, Vector3};

use crate::close_approach::{closest_approach, find_close_approaches, ScanOptions};
use crate::deflection::{DeflectionError, DeflectionScenario, DvSolveTol};
use crate::epoch::Epoch;
use crate::geometry::BPlaneEncounter;
use crate::keyhole::{CircleBranch, OpikFrame, Resonance, ResonantCircle, AU_M, JULIAN_YEAR_S};
use crate::perturber_field::EphemerisPerturber;
use crate::scenario::{RealFieldScenario, ScenarioError};

/// Metres per kilometre — the ephemeris speaks km, everything here speaks m.
const M_PER_KM: f64 = 1.0e3;

/// The aiming context a shot is flown in: the frame the target was chosen in,
/// the body radius the encounters are reduced against, and how the impulse is
/// applied.
///
/// A struct rather than four more arguments because these four always travel
/// together and always come from the same nominal encounter — splitting them at
/// call sites is how a shot ends up flown in one frame and reduced against
/// another body radius.
#[derive(Debug, Clone, Copy)]
pub struct KeyholeAiming<'a> {
    /// Encounter 1's Öpik frame, built from the **nominal** encounter. Only the
    /// first flyby is expressed in it; the return gets its own (module doc).
    pub frame: &'a OpikFrame,
    /// Earth's radius, metres — the `R⊕` every b-plane reduction here is taken
    /// against, so the capture radii and hit verdicts match the nominal's.
    pub earth_radius_m: f64,
    /// When the impulse is applied.
    pub deflection_epoch: Epoch,
    /// Which way it points (need not be unit; the zero vector is refused).
    pub direction: Vector3<f64>,
}

/// Why a keyhole shot could not be aimed or flown.
#[derive(Debug, Clone)]
pub enum KeyholeTargetError {
    /// This encounter cannot reach the resonance at any impact parameter — no
    /// `cos θ'` produces that `a'`, so there is no circle to aim at.
    ResonanceOutOfReach(Resonance),
    /// The circle exists but does not extend to the requested `ξ`: the vertical
    /// line misses it. Aiming from this ξ is impossible for this resonance, which
    /// is a fact about the geometry and not a numerical failure — a bare
    /// `√(R² − ξ²)` would have returned NaN here.
    XiOffCircle {
        /// The resonance asked for.
        resonance: Resonance,
        /// The ξ held fixed, metres.
        xi: f64,
        /// The circle's radius, metres — `|xi|` must not exceed it.
        radius: f64,
    },
    /// The aimed deflection produced no Earth encounter inside the scan gate, so
    /// there is no first flyby to set up a return.
    NoFirstEncounter {
        /// The impulse magnitude tried, m/s.
        dv_m_s: f64,
    },
    /// The flight found no Earth approach inside the return gate.
    NoReturn {
        /// The impulse magnitude tried, m/s.
        dv_m_s: f64,
        /// The gate searched, metres.
        gate_m: f64,
    },
    /// A deflection solve or evaluation failed.
    Deflection(DeflectionError),
    /// A propagation or scenario operation failed.
    Scenario(String),
    /// An input was not usable (non-finite, non-positive where it must be
    /// positive, and so on).
    InvalidInput(String),
}

impl std::fmt::Display for KeyholeTargetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KeyholeTargetError::ResonanceOutOfReach(r) => {
                write!(f, "{r} is out of reach of this encounter at any b")
            }
            KeyholeTargetError::XiOffCircle {
                resonance,
                xi,
                radius,
            } => write!(
                f,
                "{resonance}'s circle has radius {:.0} km, so it never reaches ξ = {:.0} km",
                radius / 1e3,
                xi / 1e3
            ),
            KeyholeTargetError::NoFirstEncounter { dv_m_s } => write!(
                f,
                "Δv {dv_m_s:.6} m/s left no Earth encounter inside the scan gate"
            ),
            KeyholeTargetError::NoReturn { dv_m_s, gate_m } => write!(
                f,
                "Δv {dv_m_s:.6} m/s produced no return inside {:.4} AU",
                gate_m / AU_M
            ),
            KeyholeTargetError::Deflection(e) => write!(f, "deflection: {e}"),
            KeyholeTargetError::Scenario(e) => write!(f, "scenario: {e}"),
            KeyholeTargetError::InvalidInput(e) => write!(f, "invalid input: {e}"),
        }
    }
}

impl std::error::Error for KeyholeTargetError {}

impl From<DeflectionError> for KeyholeTargetError {
    fn from(e: DeflectionError) -> Self {
        KeyholeTargetError::Deflection(e)
    }
}

impl From<ScenarioError> for KeyholeTargetError {
    fn from(e: ScenarioError) -> Self {
        KeyholeTargetError::Scenario(e.to_string())
    }
}

/// Where on a resonant circle to fly, and the perigee that gets you there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyholeAim {
    /// The circle aimed at.
    pub circle: ResonantCircle,
    /// Which of the two crossings at this ξ.
    pub branch: CircleBranch,
    /// The chosen b-plane point, `(ξ, ζ)` metres.
    pub target: Vector2<f64>,
    /// `|B|` at that point, metres.
    pub impact_parameter_m: f64,
    /// The gravitationally-focused perigee that corresponds to it, metres — what
    /// [`DeflectionScenario::required_dv`] takes as its target. Below Earth's
    /// radius this aim is an *impact*, not a keyhole; the caller decides whether
    /// that is interesting (it usually is: most resonant circles pass through the
    /// capture disc) but a deflection solver cannot aim at it.
    pub perigee_m: f64,
}

/// Choose the point of `resonance`'s circle to aim at, holding `xi` fixed.
///
/// `xi` is normally the ξ the deflection naturally lands on — the nominal
/// encounter's own, which a small along-track nudge barely moves — so this picks
/// *which ζ* to fly to and lets the deflection provide the rest. Both crossings
/// are offered ([`CircleBranch`]) because they are genuinely different returns,
/// and the sign of `xi` is the caller's ("either ξ side" is a property of the
/// deflection direction, prograde or retrograde, and is measured rather than
/// chosen).
///
/// Closed-form: microseconds. Errors are geometric facts, not numerical ones —
/// see [`KeyholeTargetError::ResonanceOutOfReach`] and
/// [`KeyholeTargetError::XiOffCircle`].
pub fn aim_at_resonance(
    frame: &OpikFrame,
    resonance: Resonance,
    xi: f64,
    branch: CircleBranch,
) -> Result<KeyholeAim, KeyholeTargetError> {
    if !xi.is_finite() {
        return Err(KeyholeTargetError::InvalidInput(format!("ξ = {xi}")));
    }
    let circle = frame
        .resonant_circle(resonance)
        .ok_or(KeyholeTargetError::ResonanceOutOfReach(resonance))?;
    let target = circle
        .point_at_xi(xi, branch)
        .ok_or(KeyholeTargetError::XiOffCircle {
            resonance,
            xi,
            radius: circle.radius,
        })?;
    let b = target.norm();
    Ok(KeyholeAim {
        circle,
        branch,
        target,
        impact_parameter_m: b,
        perigee_m: frame.perigee_for_impact_parameter(b),
    })
}

/// Knobs for flying a shot. The defaults reproduce `probe_keyhole_return`'s
/// measured run on the shipping rock; every one of them is a cost/coverage
/// trade, so they are a struct rather than constants buried in the body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyholeShotOptions {
    /// Scan gate for **encounter 1**, metres. The shipping scenario uses 5e8.
    pub first_scan: ScanOptions,
    /// How far past encounter 1 to hand the trajectory over to the free
    /// propagation, seconds. Must clear Earth's sphere of influence — 30 days is
    /// ~2 orders of magnitude outside it.
    pub handoff_seconds_after_ca: f64,
    /// How long past the handoff to fly, seconds. `None` derives it from the
    /// resonance: `h` years plus 0.6 yr of margin, which is what the closed
    /// form's timing error demands.
    pub return_span_seconds: Option<f64>,
    /// Snapshot cadence of the return propagation, seconds.
    pub return_cadence_seconds: f64,
    /// Census gate for the **return**, metres. Deliberately wide (0.05 AU by
    /// default): the closed form can be ~10⁶ km out in arrival placement, so a
    /// shipping-sized gate would report "no return" for a return that is there.
    pub return_gate_m: f64,
    /// Sampling cadence and time tolerance of the return census.
    pub return_scan_max_sample_dt: f64,
}

impl Default for KeyholeShotOptions {
    fn default() -> Self {
        Self {
            first_scan: ScanOptions {
                max_sample_dt: 6.0 * 3600.0,
                time_tol_seconds: 1.0e-3,
                max_distance: Some(5.0e8),
            },
            handoff_seconds_after_ca: 30.0 * 86_400.0,
            return_span_seconds: None,
            return_cadence_seconds: 86_400.0,
            return_gate_m: 0.05 * AU_M,
            return_scan_max_sample_dt: 6.0 * 3600.0,
        }
    }
}

impl KeyholeShotOptions {
    /// The flight span for a given resonance, seconds — the explicit
    /// `return_span_seconds` if set, else `h` years plus 0.6 yr of margin.
    pub fn span_for(&self, resonance: Resonance) -> f64 {
        self.return_span_seconds
            .unwrap_or((resonance.h as f64 + 0.6) * JULIAN_YEAR_S)
    }
}

/// The return encounter as flown, reduced in **its own** Öpik frame.
///
/// See the module doc for why the frame is rebuilt here rather than reusing
/// encounter 1's: `h` years later Earth is elsewhere and the asteroid arrives on
/// a different asymptote, so the first frame's axes carry no meaning, and it is
/// the ξ/ζ split *in this frame* that says whether a refinement converged.
#[derive(Debug, Clone, PartialEq)]
pub struct FlownReturn {
    /// When the return closest approach happens.
    pub epoch: Epoch,
    /// Geocentric distance at that closest approach, metres.
    pub distance_m: f64,
    /// How long after encounter 1, Julian years — compare against the resonance's
    /// `h` to confirm the return is the one that was aimed at.
    pub years_after_first: f64,
    /// How many approaches the census found inside the gate (more than one means
    /// the gate caught neighbours of the intended return).
    pub approaches_in_gate: usize,
    /// The return's b-plane reduction, or `None` if the approach was not a
    /// hyperbolic flyby (a very slow pass has no `v_inf`).
    pub encounter: Option<BPlaneEncounter>,
    /// `ξ` of the return's b-vector **in the return's own frame**, metres — the
    /// spatial offset between the two orbits, which no timing change removes.
    /// `None` if the frame or the reduction could not be built.
    pub xi_m: Option<f64>,
    /// `ζ` in the return's own frame, metres — the *timing* coordinate. A
    /// converged refinement drives this toward zero.
    pub zeta_m: Option<f64>,
}

impl FlownReturn {
    /// `|ζ| / |ξ|` — the share of the miss that is still timing. Small means the
    /// search has converged onto the spatial floor; large means it has not, and
    /// the scalar distance cannot tell you which. `None` when the return had no
    /// b-plane reduction.
    pub fn timing_share(&self) -> Option<f64> {
        let (xi, zeta) = (self.xi_m?, self.zeta_m?);
        Some(zeta.abs() / xi.abs().max(f64::MIN_POSITIVE))
    }
}

/// One flown shot: an impulse, the flyby it produces, and the return it sets up.
#[derive(Debug, Clone, PartialEq)]
pub struct KeyholeShot {
    /// The impulse magnitude flown, m/s (along the caller's direction).
    pub dv_m_s: f64,
    /// Encounter 1's b-plane reduction.
    pub encounter: BPlaneEncounter,
    /// Encounter 1's b-point in the **aiming** frame, `(ξ, ζ)` metres.
    pub point: Vector2<f64>,
    /// The closed form's `a'` at that point, metres — what the map predicts this
    /// flyby leaves the rock on, to be read against the resonance it aimed at.
    pub a_prime_m: f64,
    /// The return, or `None` if none fell inside the gate.
    pub flown_return: Option<FlownReturn>,
}

impl KeyholeShot {
    /// The return distance, or infinity when there was no return — the scalar the
    /// refinement minimises. Infinity rather than `None` so a search can compare
    /// it without special-casing, and because "no return in a 0.05 AU gate" really
    /// is worse than any return inside it.
    pub fn return_distance_m(&self) -> f64 {
        self.flown_return
            .as_ref()
            .map_or(f64::INFINITY, |r| r.distance_m)
    }
}

/// Fly one impulse and measure the return it sets up.
///
/// `frame` is the aiming frame (encounter 1's Öpik frame, from the *nominal*
/// encounter) — used only to express encounter 1's b-point and closed-form `a'`
/// in the same coordinates the aim was chosen in. The return gets its own frame.
///
/// ~15 s on the shipping rock in release: a campaign re-fly plus an `h`-year
/// propagation.
pub fn fly_keyhole_shot(
    scenario: &RealFieldScenario,
    ds: &DeflectionScenario<'_>,
    aiming: KeyholeAiming<'_>,
    dv_m_s: f64,
    resonance: Resonance,
    opts: &KeyholeShotOptions,
) -> Result<KeyholeShot, KeyholeTargetError> {
    if !(dv_m_s.is_finite() && dv_m_s >= 0.0) {
        return Err(KeyholeTargetError::InvalidInput(format!(
            "dv must be finite and ≥ 0 (got {dv_m_s})"
        )));
    }
    let dir_norm = aiming.direction.norm();
    if !(dir_norm.is_finite() && dir_norm > 0.0) {
        return Err(KeyholeTargetError::Deflection(DeflectionError::NoDirection));
    }
    let frame = aiming.frame;
    let dir = aiming.direction / dir_norm;

    let (clock, _) = ds.deflected_trajectory(aiming.deflection_epoch, dv_m_s * dir)?;
    let eph = scenario.ephemeris().clone();
    let earth = EphemerisPerturber::new(eph.clone(), EARTH_J2000);
    let ca1 = closest_approach(&clock, &earth, opts.first_scan)
        .map_err(|e| KeyholeTargetError::Scenario(e.to_string()))?
        .ok_or(KeyholeTargetError::NoFirstEncounter { dv_m_s })?;
    let enc1 = ca1
        .b_plane(frame.mu_earth, aiming.earth_radius_m)
        .map_err(|e| KeyholeTargetError::Scenario(e.to_string()))?;
    let point = frame.project(&enc1.b_vector);

    // Hand off well outside Earth's sphere of influence, then fly the resonant
    // orbit forward under the same field.
    let t_hand = ca1.epoch.shifted_by_seconds(opts.handoff_seconds_after_ca);
    let hand = clock
        .state_at(t_hand)
        .map_err(|e| KeyholeTargetError::Scenario(e.to_string()))?;
    let span = opts.span_for(resonance);
    let snapshots = (span / opts.return_cadence_seconds).ceil().max(2.0) as u32;
    let onward = scenario.propagate_free(t_hand, hand, opts.return_cadence_seconds, snapshots)?;
    let returns = find_close_approaches(
        &onward,
        &earth,
        ScanOptions {
            max_sample_dt: opts.return_scan_max_sample_dt,
            time_tol_seconds: opts.first_scan.time_tol_seconds,
            max_distance: Some(opts.return_gate_m),
        },
    )
    .map_err(|e| KeyholeTargetError::Scenario(e.to_string()))?;

    let flown_return = returns
        .iter()
        .min_by(|a, b| {
            a.distance
                .partial_cmp(&b.distance)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|best| {
            let encounter = best.b_plane(frame.mu_earth, aiming.earth_radius_m).ok();
            // The return's OWN frame — see the module doc. Earth's heliocentric
            // state at the *return* epoch, not encounter 1's.
            let own = encounter.as_ref().and_then(|e| {
                let (r_km, v_km) = eph
                    .state_km_s(EARTH_J2000, SUN_J2000, best.epoch.as_hifitime())
                    .ok()?;
                let mu_sun = eph.sun_gm_m3_s2().ok()?;
                let f2 = OpikFrame::new(e, r_km * M_PER_KM, v_km * M_PER_KM, mu_sun).ok()?;
                Some(f2.project(&e.b_vector))
            });
            FlownReturn {
                epoch: best.epoch,
                distance_m: best.distance,
                years_after_first: (best.epoch.tdb_seconds_past_j2000()
                    - ca1.epoch.tdb_seconds_past_j2000())
                    / JULIAN_YEAR_S,
                approaches_in_gate: returns.len(),
                encounter,
                xi_m: own.map(|p| p.x),
                zeta_m: own.map(|p| p.y),
            }
        });

    Ok(KeyholeShot {
        dv_m_s,
        a_prime_m: frame.post_encounter_semi_major_axis(point),
        encounter: enc1,
        point,
        flown_return,
    })
}

/// How hard to refine, and how the bracket is grown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyholeRefineTol {
    /// Initial bracket half-width as a fraction of the aimed Δv.
    pub bracket_fraction: f64,
    /// How many times the bracket may be widened before giving up on containing
    /// the minimum.
    pub max_widenings: usize,
    /// Golden-section iterations.
    pub max_iterations: usize,
    /// Stop when the bracket is this fraction of the aimed Δv.
    pub rel_tol: f64,
}

impl Default for KeyholeRefineTol {
    fn default() -> Self {
        Self {
            bracket_fraction: 0.01,
            max_widenings: 6,
            max_iterations: 12,
            rel_tol: 1.0e-7,
        }
    }
}

/// A refined keyhole shot: the aim, the flight at the aimed Δv, and the flight
/// at the Δv that minimises the return distance.
#[derive(Debug, Clone, PartialEq)]
pub struct KeyholeSolution {
    /// Where on the circle the aim pointed.
    pub aim: KeyholeAim,
    /// The impulse the closed-form aim asked for, m/s.
    pub aim_dv_m_s: f64,
    /// The flight at that impulse — the map's own answer, before refinement.
    pub aimed: KeyholeShot,
    /// The flight at the refined impulse.
    pub best: KeyholeShot,
    /// The final bracket width in Δv, m/s. This is the **keyhole in Δv terms**:
    /// how finely the impulse has to be controlled to stay in the door.
    pub dv_window_m_s: f64,
    /// How many flights it took (aim flight included).
    pub flights: usize,
}

impl KeyholeSolution {
    /// Whether the refined return lands inside the capture disc — a resonant
    /// **impact** keyhole, flown, rather than a return that misses.
    pub fn is_impact_return(&self) -> bool {
        self.best
            .flown_return
            .as_ref()
            .is_some_and(|r| r.distance_m <= self.aimed.encounter.capture_radius)
    }
}

/// Aim at `resonance`, fly it, and refine the impulse until the return miss hits
/// its floor.
///
/// The whole three-step recipe: [`aim_at_resonance`] →
/// [`DeflectionScenario::required_dv`] → [`fly_keyhole_shot`] → a bracketed
/// golden-section search on the return distance. `xi` and `branch` say where on
/// the circle to aim (see [`aim_at_resonance`]); `direction` and
/// `deflection_epoch` say how the impulse is applied.
///
/// **~4 minutes on the shipping rock in release.** Never call this on a build
/// path, a frame, or anything a user is waiting on synchronously.
///
/// The minimum it finds is a *floor*, not a zero: what remains is the
/// post-encounter orbit's spatial offset at the return. [`FlownReturn::xi_m`] /
/// [`FlownReturn::zeta_m`] are how a caller checks that the floor really is
/// spatial (converged) rather than timing (not converged) — see the module doc.
#[allow(clippy::too_many_arguments)]
pub fn solve_keyhole_return(
    scenario: &RealFieldScenario,
    ds: &DeflectionScenario<'_>,
    aiming: KeyholeAiming<'_>,
    resonance: Resonance,
    xi: f64,
    branch: CircleBranch,
    opts: &KeyholeShotOptions,
    tol: KeyholeRefineTol,
    dv_tol: DvSolveTol,
) -> Result<KeyholeSolution, KeyholeTargetError> {
    let aim = aim_at_resonance(aiming.frame, resonance, xi, branch)?;
    if !(aim.perigee_m > 0.0) {
        return Err(KeyholeTargetError::InvalidInput(format!(
            "{resonance} at ξ = {:.0} km wants a perigee of {:.0} km — inside Earth's centre; \
             a deflection solver cannot aim at it",
            xi / 1e3,
            aim.perigee_m / 1e3
        )));
    }
    let aim_dv = ds.required_dv(
        aiming.deflection_epoch,
        aiming.direction,
        aim.perigee_m,
        dv_tol,
    )?;

    let mut flights = 0usize;
    let mut fly = |dv: f64| -> Result<KeyholeShot, KeyholeTargetError> {
        flights += 1;
        fly_keyhole_shot(scenario, ds, aiming, dv, resonance, opts)
    };

    let aimed = fly(aim_dv)?;
    let mut f_mid = aimed.return_distance_m();
    let mut mid = aim_dv;
    let mut lo = aim_dv * (1.0 - tol.bracket_fraction);
    let mut hi = aim_dv * (1.0 + tol.bracket_fraction);
    let mut f_lo = fly(lo)?.return_distance_m();
    let mut f_hi = fly(hi)?.return_distance_m();

    // Widen until the centre is the lowest of the three: the miss is ~linear in
    // |Δv − Δv*| on each side, so a mis-centred bracket walks downhill fast.
    for _ in 0..tol.max_widenings {
        if f_mid <= f_lo && f_mid <= f_hi {
            break;
        }
        if f_lo < f_mid {
            hi = mid;
            f_hi = f_mid;
            mid = lo;
            f_mid = f_lo;
            lo = mid - (hi - mid);
            f_lo = fly(lo)?.return_distance_m();
        } else {
            lo = mid;
            f_lo = f_mid;
            mid = hi;
            f_mid = f_hi;
            hi = mid + (mid - lo);
            f_hi = fly(hi)?.return_distance_m();
        }
    }

    // Golden-section on [lo, hi].
    let phi = 0.5 * (5.0f64.sqrt() - 1.0);
    let (mut a, mut b) = (lo, hi);
    let mut x1 = b - phi * (b - a);
    let mut x2 = a + phi * (b - a);
    let mut f1 = fly(x1)?.return_distance_m();
    let mut f2 = fly(x2)?.return_distance_m();
    for _ in 0..tol.max_iterations {
        if f1 < f2 {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - phi * (b - a);
            f1 = fly(x1)?.return_distance_m();
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + phi * (b - a);
            f2 = fly(x2)?.return_distance_m();
        }
        if (b - a) < tol.rel_tol * aim_dv.max(f64::MIN_POSITIVE) {
            break;
        }
    }
    let dv_best = if f1 < f2 { x1 } else { x2 };
    let best = fly(dv_best)?;

    Ok(KeyholeSolution {
        aim,
        aim_dv_m_s: aim_dv,
        aimed,
        best,
        dv_window_m_s: b - a,
        flights,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::EARTH_EQUATORIAL_RADIUS_M;
    use crate::keyhole::perigee_state_for_asymptote;

    const MU_EARTH: f64 = 3.986_004_356e14;
    const MU_SUN: f64 = 1.327_124_400_41e20;

    /// A frame like the shipping encounter's: Earth on a circular 1 AU orbit,
    /// a 5 km/s asymptote 30° off Earth's motion.
    fn frame() -> OpikFrame {
        let r_e = Vector3::new(AU_M, 0.0, 0.0);
        let v_circ = (MU_SUN / AU_M).sqrt();
        let v_e = Vector3::new(0.0, v_circ, 0.0);
        // Ŝ at 30° from V⊕, in the orbital plane.
        let s_hat = Vector3::new(-0.5, 3.0f64.sqrt() / 2.0, 0.0);
        let b_hat = Vector3::new(3.0f64.sqrt() / 2.0, 0.5, 0.0);
        let (r, v) = perigee_state_for_asymptote(5_000.0, 5.0e7, s_hat, b_hat, MU_EARTH);
        let enc = BPlaneEncounter::from_relative_state(r, v, MU_EARTH, EARTH_EQUATORIAL_RADIUS_M)
            .unwrap();
        OpikFrame::new(&enc, r_e, v_e, MU_SUN).unwrap()
    }

    /// Aiming is closed-form and instant: both branches land exactly on the
    /// circle, at exactly the requested ξ, and the perigee round-trips.
    #[test]
    fn both_branches_land_on_the_circle_at_the_requested_xi() {
        let f = frame();
        let circles = f.resonant_circles(2..=10, 24, 200.0 * f.capture_radius);
        assert!(!circles.is_empty(), "the test frame reaches some resonance");
        let mut checked = 0;
        for c in &circles {
            if c.radius < 1.0e3 {
                continue;
            }
            let xi = 0.4 * c.radius;
            let minus = aim_at_resonance(&f, c.resonance, xi, CircleBranch::Minus).unwrap();
            let plus = aim_at_resonance(&f, c.resonance, xi, CircleBranch::Plus).unwrap();
            assert!(plus.target.y > minus.target.y, "Plus must be the upper ζ");
            for aim in [minus, plus] {
                assert!((aim.target.x - xi).abs() < 1e-6 * c.radius);
                assert!(
                    aim.circle.signed_distance(aim.target).abs() < 1e-6 * c.radius,
                    "the aim point must be ON the circle"
                );
                assert!(
                    (aim.impact_parameter_m - aim.target.norm()).abs() < 1e-9 * aim.target.norm()
                );
                // b ↔ perigee round-trips through the focusing conversion.
                let back = f.impact_parameter_for_perigee(aim.perigee_m);
                assert!(
                    (back - aim.impact_parameter_m).abs() < 1e-6 * aim.impact_parameter_m,
                    "perigee {:.1} km → b {:.3} km ≠ {:.3} km",
                    aim.perigee_m / 1e3,
                    back / 1e3,
                    aim.impact_parameter_m / 1e3
                );
                // And the closed form really puts a' on the resonance there.
                let a = f.post_encounter_semi_major_axis(aim.target);
                assert!(
                    (a - c.a_prime).abs() < 1e-9 * c.a_prime,
                    "{}: a' {:.9} AU ≠ {:.9} AU",
                    c.resonance,
                    a / AU_M,
                    c.a_prime / AU_M
                );
                checked += 1;
            }
        }
        assert!(checked >= 4, "only {checked} aims exercised");
    }

    /// An out-of-reach ξ is a **geometric fact**, reported as such. The bare
    /// `√(R² − ξ²)` the probe used would have produced NaN here and carried it
    /// silently into a perigee, a Δv solve and a four-minute flight.
    #[test]
    fn an_xi_wider_than_the_circle_is_refused_not_nanned() {
        let f = frame();
        let c = f
            .resonant_circles(2..=10, 24, 200.0 * f.capture_radius)
            .into_iter()
            .find(|c| c.radius > 1.0e3)
            .expect("a circle");
        let err = aim_at_resonance(&f, c.resonance, c.radius * 1.5, CircleBranch::Minus)
            .expect_err("ξ beyond the radius cannot be aimed at");
        match err {
            KeyholeTargetError::XiOffCircle { radius, xi, .. } => {
                assert!(xi > radius);
            }
            other => panic!("wrong error: {other}"),
        }
        // Exactly at the radius the two branches coincide — the tangent point.
        let (m, p) = c.points_at_xi(c.radius).expect("tangent");
        assert!((m - p).norm() < 1e-6 * c.radius);
    }

    /// A resonance no `cos θ'` can produce is refused before any propagation.
    #[test]
    fn an_unreachable_resonance_is_refused_before_it_costs_anything() {
        let f = frame();
        // 1:20 — a' far below anything this encounter can turn the rock onto.
        let r = Resonance { h: 1, k: 20 };
        assert!(
            f.resonant_circle(r).is_none(),
            "the test case must be out of reach"
        );
        match aim_at_resonance(&f, r, 0.0, CircleBranch::Minus) {
            Err(KeyholeTargetError::ResonanceOutOfReach(got)) => assert_eq!(got, r),
            other => panic!("expected out-of-reach, got {other:?}"),
        }
    }

    /// Kernel-gated, ~30 s. **One flight** at the Δv `probe_keyhole_return`
    /// refined to, checked for the thing a scalar return distance cannot show:
    /// *where* the residual miss lives, in the return's own Öpik frame.
    ///
    /// Δv is a timing knob. `ζ` is the timing coordinate; `ξ` is the spatial
    /// offset between the two orbits, which no timing change removes. So at a
    /// converged floor the miss has to be **mostly ξ, with ζ driven small** — and
    /// if instead the floor sits in ζ, the golden-section search simply stopped
    /// early and the "physical floor" story is a convergence bug in costume. The
    /// scalar `distance_m` is identical either way, which is exactly why this
    /// test reduces the return in its own frame instead of trusting it.
    ///
    /// **Measured (2026-09-05):** ξ₂ = 4 014 km, ζ₂ = 891 km, so the floor is
    /// 78 % spatial and the search has converged. Note that the return's *impact
    /// parameter* is 4 112 km while its geocentric closest approach is 1 141 km —
    /// gravitational focusing, and the reason the two numbers in this project's
    /// keyhole notes are not the same number.
    ///
    /// The full four-minute solve lives in `examples/probe_keyhole_return.rs`.
    /// This is the 30-second regression that keeps the answer pinned.
    #[test]
    fn the_flown_return_floor_is_spatial_not_timing() {
        use crate::deflection::along_track_unit;
        use crate::geometry::EARTH_EQUATORIAL_RADIUS_M as R_E;
        use crate::scenario::{ImpactorConfig, RealFieldScenario};

        let Some(_) = crate::kernels::resolve_for_test("the flown 3:4 keyhole floor") else {
            return;
        };
        /// The Δv `probe_keyhole_return` refined the 3:4 return down to.
        const KEYHOLE_DV_RETROGRADE_M_S: f64 = 0.216_550;

        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("build");
        let ds = sc.deflection().expect("deflection");
        let nominal = sc.nominal_hit(&ds).expect("nominal hit");
        let t_ca = ds
            .nominal_encounter_epoch()
            .expect("epoch")
            .expect("an encounter");
        let eph = sc.ephemeris().clone();
        let (r_km, v_km) = eph
            .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
            .expect("Earth state");
        let frame = OpikFrame::new(
            &nominal,
            r_km * M_PER_KM,
            v_km * M_PER_KM,
            eph.sun_gm_m3_s2().expect("sun GM"),
        )
        .expect("frame");
        let epoch0 = sc.epoch0();
        let seed = ds.nominal().state_at(epoch0).expect("seed");
        let retro = -along_track_unit(seed).expect("along-track");

        // The aim is closed-form and free: check it lands where the probe aimed
        // before spending fifteen seconds on the flight.
        let xi = frame.project(&nominal.b_vector).x;
        let aim = aim_at_resonance(&frame, Resonance { h: 3, k: 4 }, xi, CircleBranch::Minus)
            .expect("the 3:4 circle reaches the nominal's ξ");
        println!(
            "aim: ξ {:.0} ζ {:.0} km → b {:.0} km, perigee {:.0} km ({:.2} R⊕)",
            aim.target.x / M_PER_KM,
            aim.target.y / M_PER_KM,
            aim.impact_parameter_m / M_PER_KM,
            aim.perigee_m / M_PER_KM,
            aim.perigee_m / R_E
        );
        assert!(
            (150.0e6..=157.0e6).contains(&aim.impact_parameter_m),
            "the Minus branch at the nominal's ξ is the 3:4 far end, ~153 500 km; got {:.0} km",
            aim.impact_parameter_m / M_PER_KM
        );

        let aiming = KeyholeAiming {
            frame: &frame,
            earth_radius_m: nominal.earth_radius,
            deflection_epoch: epoch0,
            direction: retro,
        };
        let shot = fly_keyhole_shot(
            &sc,
            &ds,
            aiming,
            KEYHOLE_DV_RETROGRADE_M_S,
            Resonance { h: 3, k: 4 },
            &KeyholeShotOptions::default(),
        )
        .expect("the flight runs");

        let ret = shot
            .flown_return
            .as_ref()
            .expect("a return inside the gate");
        println!(
            "flown: encounter-1 b {:.0} km, closed-form a' {:.6} AU; return {:.0} km at {} \
             ({:.3} yr later, {} in gate)",
            shot.encounter.impact_parameter / M_PER_KM,
            shot.a_prime_m / AU_M,
            ret.distance_m / M_PER_KM,
            ret.epoch.as_hifitime(),
            ret.years_after_first,
            ret.approaches_in_gate
        );
        let (xi2, zeta2) = (
            ret.xi_m.expect("the return reduces in its own frame"),
            ret.zeta_m.expect("the return reduces in its own frame"),
        );
        println!(
            "return in ITS OWN Öpik frame: ξ₂ {:.0} km (spatial), ζ₂ {:.0} km (timing), \
             |ζ₂|/|ξ₂| = {:.3}",
            xi2 / M_PER_KM,
            zeta2 / M_PER_KM,
            ret.timing_share().expect("both components")
        );

        assert!(
            (2.9..=3.1).contains(&ret.years_after_first),
            "the 3:4 return must arrive ~3 yr after the flyby, got {:.3}",
            ret.years_after_first
        );
        assert!(
            ret.distance_m < 4.0 * nominal.capture_radius,
            "return miss {:.0} km — the flown keyhole has moved",
            ret.distance_m / M_PER_KM
        );
        // THE check. A converged floor is spatial: the timing component must be
        // the smaller half of it, not the larger.
        assert!(
            ret.timing_share().expect("both components") < 0.5,
            "the floor is {:.3} timing (ζ₂ {:.0} km) against spatial (ξ₂ {:.0} km), where \
             0.222 was measured — a share this large is an unconverged search, not the \
             orbit-to-orbit offset the module claims, and the scalar {:.0} km return \
             distance cannot tell the two apart",
            ret.timing_share().unwrap(),
            zeta2 / M_PER_KM,
            xi2 / M_PER_KM,
            ret.distance_m / M_PER_KM
        );
        // And the b-plane point really is the whole miss: |B|² = ξ² + ζ².
        let enc2 = ret.encounter.as_ref().expect("the return is hyperbolic");
        let b2 = (xi2 * xi2 + zeta2 * zeta2).sqrt();
        assert!(
            (b2 - enc2.impact_parameter).abs() < 1e-6 * enc2.impact_parameter,
            "the return's own (ξ, ζ) must carry its |B|"
        );
    }

    /// The span default follows the resonance rather than a constant, which is
    /// the whole point of generalising past the 3:4 probe.
    #[test]
    fn the_flight_span_follows_the_resonance() {
        let o = KeyholeShotOptions::default();
        for h in [2u32, 3, 7, 13] {
            let years = o.span_for(Resonance { h, k: h + 1 }) / JULIAN_YEAR_S;
            assert!(
                (years - (h as f64 + 0.6)).abs() < 1e-9,
                "h={h} span {years:.2} yr"
            );
        }
        let fixed = KeyholeShotOptions {
            return_span_seconds: Some(4.0 * JULIAN_YEAR_S),
            ..Default::default()
        };
        assert!((fixed.span_for(Resonance { h: 9, k: 10 }) / JULIAN_YEAR_S - 4.0).abs() < 1e-9);
    }
}
