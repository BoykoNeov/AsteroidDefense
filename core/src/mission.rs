//! `mission` — the porkchop / deliverability layer (HANDOFF §7, §8).
//!
//! This is where a Lambert transfer ([`crate::lambert`]) and a real launcher
//! ([`crate::launch_vehicle`]) meet the deflection physics ([`crate::deflection`])
//! to answer the question the MVP curve assumes away: **can a spacecraft actually
//! be delivered to nudge this asteroid, and does the nudge — aimed by the real
//! arrival geometry, not an idealized along-track push — turn the hit into a
//! miss?** (§7, §180.)
//!
//! # Two layers, by cost — the same split the live force menu uses
//! Coupling the impulse *direction* to the Lambert arrival geometry means a true
//! deflection check needs a **full-field re-propagation per launch window**, which
//! is far too expensive to run over every cell of a porkchop grid. So the layer
//! splits exactly as the on-demand force menu does (cheap-always-on,
//! expensive-on-demand):
//!
//! - **The cheap grid** ([`porkchop_grid`]) is pure scalar math over precomputed
//!   Earth and asteroid state arrays: per cell it solves one Lambert transfer and
//!   records the departure `C3`, the arrival relative speed, and the
//!   **along-track projection** of the impact — a first-order *effectiveness*
//!   proxy computed for free from `v2` and the asteroid's velocity. It is
//!   deliberately **vehicle-independent**: `C3` maps to deliverable mass per
//!   chosen launcher afterwards, so switching launchers never re-solves Lambert.
//! - **The on-demand verify** ([`verify_cell`], [`required_impactor_mass`]) takes
//!   *one* selected cell and re-propagates the asteroid in the full `n`-body field
//!   after the real vector impulse, reading the exact b-plane perigee.
//!
//! # deliverable ≠ well-aimed — the whole point of coupling direction
//! The along-track proxy exists because a launch window can carry plenty of `|Δv|`
//! and still barely deflect: the kinetic impactor imparts its momentum along the
//! **arrival relative velocity**, and only the component of that along the
//! asteroid's orbital track efficiently changes its semi-major axis (the §5
//! along-track lever the headline curve optimizes). A cell whose geometry meets
//! the asteroid nearly head-on or radially projects poorly onto the track — the
//! proxy surfaces that on the cheap grid, and the on-demand full-field solve
//! confirms it with the true perigee.
//!
//! # Honesty framing (the caller's contract, upheld here)
//! Endpoints are real — Earth from the ephemeris, the asteroid from its integrated
//! nominal trajectory; the two-body Lambert arc only *sizes and aims* the delivery
//! and never replaces the full-field propagation that decides hit/miss; and every
//! quantity here is a **patched-conic planning estimate**. Delivered mass is
//! modelled *as* impactor mass (no bus/propellant bookkeeping — a Phase-3
//! refinement, §8).

use std::sync::Arc;

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use nalgebra::Vector3;

use crate::deflection::{DeflectionError, DeflectionScenario};
use crate::epoch::Epoch;
use crate::geometry::BPlaneEncounter;
use crate::lambert::{lambert_universal, LambertError};
use crate::launch_vehicle::LaunchVehicle;
use crate::perturber_field::EphemerisPerturber;
use crate::scenario::RealFieldScenario;
use crate::state::StateVector;

/// SI conversion for `C3`: 1 (m/s)² = 1e-6 (km/s)². The launch-vehicle tables are
/// keyed in km²/s² (their native unit); this crossing is explicit because a silent
/// km/m slip is the classic delivery bug.
const M2_S2_TO_KM2_S2: f64 = 1.0e-6;

/// The transfer/impact metrics for one launch window, computed once from the
/// Lambert solution and reused for every launcher — all vehicle-independent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransferMetrics {
    /// Departure characteristic energy `C3 = |v1 − v⊕|²`, **km²/s²** — what the
    /// launcher tables are indexed by.
    pub c3_km2_s2: f64,
    /// Arrival relative speed `|v2 − v_ast|`, m/s — the kinetic impactor's impact
    /// speed, which sizes its momentum.
    pub arrival_v_rel_ms: f64,
    /// The along-track projection `(v2 − v_ast)·v̂_ast`, m/s (signed). The
    /// first-order effectiveness proxy: the fraction of the impact that pushes
    /// along the asteroid's orbital track. Near zero ⇒ a poorly-aimed window even
    /// if `arrival_v_rel_ms` is large.
    pub along_track_proj_ms: f64,
    /// The full arrival relative velocity `v2 − v_ast`, m/s (SSB/heliocentric —
    /// the difference is frame-invariant). This is the **impact direction**: the
    /// on-demand verify imparts the impulse along this vector.
    pub v_rel_vec: Vector3<f64>,
}

/// One cell of the porkchop grid: either no transfer exists (a Lambert gap or a
/// non-positive time of flight — rendered as an empty cell, never a `NaN`), or a
/// feasible transfer with its vehicle-independent metrics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PorkchopCell {
    /// No single-rev transfer for this (launch, arrival) pair: the arrival is at
    /// or before launch, or the geometry is degenerate / non-converging.
    NoTransfer,
    /// A transfer exists; its launch-energy and impact metrics.
    Transfer(TransferMetrics),
}

/// A launch-date × arrival-date porkchop: the [`PorkchopCell`] metrics over a
/// grid, plus the epoch axes they were sampled on. Row-major `[launch][arrival]`.
#[derive(Debug, Clone)]
pub struct Porkchop {
    /// Launch epochs (the grid's first axis).
    pub launch_epochs: Vec<Epoch>,
    /// Arrival / intercept epochs (the grid's second axis).
    pub arrival_epochs: Vec<Epoch>,
    /// `cells[i][j]` is the transfer from `launch_epochs[i]` to
    /// `arrival_epochs[j]`.
    pub cells: Vec<Vec<PorkchopCell>>,
}

/// What a chosen launcher makes of one transfer cell: the mass it can deliver at
/// that cell's `C3`, and the along-track Δv that mass would impart (the
/// effectiveness proxy scaled by the *real* deliverable mass).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellDelivery {
    /// Deliverable impactor mass at this cell's `C3`, kg. `0` = the launcher
    /// cannot reach this launch energy (infeasible cell).
    pub payload_kg: f64,
    /// The along-track Δv the delivered impactor imparts,
    /// `β·(m_sc/M)·along_track_proj`, m/s — the deliverability-weighted
    /// effectiveness proxy. `0` when the cell is infeasible.
    pub along_track_dv_ms: f64,
    /// Whether the launcher can reach this cell at all (`payload_kg > 0`).
    pub feasible: bool,
}

/// Why building or verifying a porkchop failed (as distinct from a per-cell
/// "no transfer", which is an ordinary [`PorkchopCell::NoTransfer`], not an error).
#[derive(Debug, Clone)]
pub enum MissionError {
    /// An ephemeris lookup (Earth/Sun state, or `μ_sun`) failed.
    Ephemeris(String),
    /// Building the deflection scenario or sampling the nominal trajectory failed.
    Deflection(DeflectionError),
    /// An input was out of range (empty axes, non-positive mass/β, etc.).
    InvalidInput(String),
}

impl std::fmt::Display for MissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MissionError::Ephemeris(m) => write!(f, "ephemeris lookup failed: {m}"),
            MissionError::Deflection(e) => write!(f, "deflection scenario failed: {e}"),
            MissionError::InvalidInput(m) => write!(f, "invalid mission input: {m}"),
        }
    }
}

impl std::error::Error for MissionError {}

impl From<DeflectionError> for MissionError {
    fn from(e: DeflectionError) -> Self {
        MissionError::Deflection(e)
    }
}

// --- Pure metrics (kernel-free, unit-tested in isolation) --------------------

/// Transfer + impact metrics for one launch window, from the two *heliocentric*
/// endpoint states: Earth at launch (`r1`, `v⊕`) and the asteroid at arrival
/// (`r2`, `v_ast`). `Ok(None)` when Lambert has no single-rev transfer (a grid
/// gap); `Err` only for a genuinely invalid input (non-positive `tof`/`μ`).
///
/// The along-track direction `v̂_ast` is the asteroid's *heliocentric* prograde —
/// the direction along its orbit about the Sun, which is the track the §5 lever
/// acts on.
pub fn transfer_metrics(
    earth_helio: StateVector,
    asteroid_helio: StateVector,
    tof_seconds: f64,
    mu_sun: f64,
    prograde: bool,
) -> Result<Option<TransferMetrics>, LambertError> {
    let sol = match lambert_universal(
        earth_helio.position,
        asteroid_helio.position,
        tof_seconds,
        mu_sun,
        prograde,
    ) {
        Ok(s) => s,
        // A degenerate/non-converging geometry is a grid gap, not an error.
        Err(LambertError::DegenerateGeometry { .. }) | Err(LambertError::NonConvergence { .. }) => {
            return Ok(None)
        }
        Err(e @ LambertError::InvalidInput { .. }) => return Err(e),
    };

    let v_inf_departure = sol.v1 - earth_helio.velocity;
    let c3_km2_s2 = v_inf_departure.norm_squared() * M2_S2_TO_KM2_S2;

    let v_rel_vec = sol.v2 - asteroid_helio.velocity;
    let arrival_v_rel_ms = v_rel_vec.norm();

    let v_ast_speed = asteroid_helio.velocity.norm();
    let along_track_proj_ms = if v_ast_speed > 0.0 {
        v_rel_vec.dot(&asteroid_helio.velocity) / v_ast_speed
    } else {
        0.0
    };

    Ok(Some(TransferMetrics {
        c3_km2_s2,
        arrival_v_rel_ms,
        along_track_proj_ms,
        v_rel_vec,
    }))
}

/// The impulse a kinetic impactor imparts to the asteroid:
/// `Δv = β · (m_sc / M) · v_rel_vec` (m/s, added to the asteroid's velocity).
///
/// Direction is the **real arrival relative velocity** (the coupled-direction
/// model, §3 of the mission design): the momentum transfers along the impact,
/// not along an idealized track. `β ≥ 1` is the ejecta momentum enhancement
/// (DART measured ≈ 3.6).
pub fn impact_impulse(
    v_rel_vec: Vector3<f64>,
    beta: f64,
    impactor_mass_kg: f64,
    asteroid_mass_kg: f64,
) -> Vector3<f64> {
    beta * (impactor_mass_kg / asteroid_mass_kg) * v_rel_vec
}

/// What a launcher makes of one transfer cell: deliverable mass at its `C3`, and
/// the along-track Δv that mass imparts. Pure — the vehicle-mapping half of the
/// vehicle-independent grid.
pub fn cell_delivery(
    metrics: &TransferMetrics,
    vehicle: &LaunchVehicle,
    beta: f64,
    asteroid_mass_kg: f64,
) -> CellDelivery {
    let payload_kg = vehicle.payload_kg(metrics.c3_km2_s2);
    let feasible = payload_kg > 0.0;
    let along_track_dv_ms = if feasible && asteroid_mass_kg > 0.0 {
        beta * (payload_kg / asteroid_mass_kg) * metrics.along_track_proj_ms
    } else {
        0.0
    };
    CellDelivery {
        payload_kg,
        along_track_dv_ms,
        feasible,
    }
}

// --- The grid (needs the ephemeris + the nominal trajectory) -----------------

fn helio(sun_ssb: StateVector, body_ssb: StateVector) -> StateVector {
    StateVector::new(
        body_ssb.position - sun_ssb.position,
        body_ssb.velocity - sun_ssb.velocity,
    )
}

/// Build a launch-date × arrival-date porkchop over a real scenario: for every
/// (launch, arrival) pair with `arrival − launch ≥ min_tof_seconds`, solve the
/// heliocentric Lambert transfer and record its vehicle-independent metrics.
///
/// Endpoints are real: Earth from the scenario's ephemeris, the asteroid from its
/// integrated nominal trajectory (the *pre-deflection* orbit — the spacecraft
/// rendezvous with the undeflected asteroid). Earth and asteroid states are
/// looked up **once per epoch** and reused across the grid, so the `N×M` cells
/// are pure scalar Lambert solves, not `N×M` ephemeris queries.
pub fn porkchop_grid(
    scenario: &RealFieldScenario,
    launch_epochs: &[Epoch],
    arrival_epochs: &[Epoch],
    min_tof_seconds: f64,
    prograde: bool,
) -> Result<Porkchop, MissionError> {
    if launch_epochs.is_empty() || arrival_epochs.is_empty() {
        return Err(MissionError::InvalidInput(
            "launch_epochs and arrival_epochs must be non-empty".into(),
        ));
    }

    let eph = scenario.ephemeris();
    let mu_sun = eph
        .sun_gm_m3_s2()
        .map_err(|e| MissionError::Ephemeris(e.to_string()))?;

    let sun = EphemerisPerturber::new(Arc::clone(eph), SUN_J2000);
    let earth = EphemerisPerturber::new(Arc::clone(eph), EARTH_J2000);

    // Precompute Earth heliocentric states at each launch epoch.
    let mut earth_helio = Vec::with_capacity(launch_epochs.len());
    for &t in launch_epochs {
        let sun_ssb = sun
            .state_at(t)
            .map_err(|e| MissionError::Ephemeris(e.to_string()))?;
        let earth_ssb = earth
            .state_at(t)
            .map_err(|e| MissionError::Ephemeris(e.to_string()))?;
        earth_helio.push(helio(sun_ssb, earth_ssb));
    }

    // Precompute asteroid heliocentric states at each arrival epoch, from the
    // cached nominal trajectory. `deflection()` propagates the nominal once and
    // reuses it (§ scenario), so this does not re-fly the cruise.
    let ds = scenario.deflection()?;
    let mut ast_helio = Vec::with_capacity(arrival_epochs.len());
    for &t in arrival_epochs {
        let sun_ssb = sun
            .state_at(t)
            .map_err(|e| MissionError::Ephemeris(e.to_string()))?;
        // Outside the propagated span there is no asteroid to intercept: leave a
        // sentinel that makes every cell at this arrival a NoTransfer.
        let ast = match ds.nominal().state_at(t) {
            Ok(s) => Some(helio(sun_ssb, s)),
            Err(_) => None,
        };
        ast_helio.push(ast);
    }

    let mut cells = Vec::with_capacity(launch_epochs.len());
    for (i, &t_l) in launch_epochs.iter().enumerate() {
        let mut row = Vec::with_capacity(arrival_epochs.len());
        for (j, &t_a) in arrival_epochs.iter().enumerate() {
            let tof = t_a.tdb_seconds_past_j2000() - t_l.tdb_seconds_past_j2000();
            let cell = match (&ast_helio[j], tof >= min_tof_seconds) {
                (Some(ast), true) => {
                    match transfer_metrics(earth_helio[i], *ast, tof, mu_sun, prograde) {
                        Ok(Some(m)) => PorkchopCell::Transfer(m),
                        // Lambert gap or (guarded above) invalid input → no cell.
                        Ok(None) | Err(_) => PorkchopCell::NoTransfer,
                    }
                }
                _ => PorkchopCell::NoTransfer,
            };
            row.push(cell);
        }
        cells.push(row);
    }

    Ok(Porkchop {
        launch_epochs: launch_epochs.to_vec(),
        arrival_epochs: arrival_epochs.to_vec(),
        cells,
    })
}

// --- On-demand full-field verify (one selected cell) -------------------------

/// Re-propagate the asteroid in the **full `n`-body field** after the real vector
/// impulse for one selected cell, and return the resulting Earth encounter — the
/// exact b-plane perigee behind the grid's along-track proxy.
///
/// The impulse is applied at `arrival_epoch` (when the spacecraft reaches the
/// asteroid), aimed along the cell's `v_rel_vec` (the coupled-direction model) and
/// sized by `β·(m_sc/M)·|v_rel|`. `Ok(None)` is a clean miss (the deflected pass
/// left the scan gate). Expensive — one propagation — so it runs per *selected*
/// cell, never across the grid.
pub fn verify_cell(
    deflection: &DeflectionScenario,
    arrival_epoch: Epoch,
    metrics: &TransferMetrics,
    beta: f64,
    impactor_mass_kg: f64,
    asteroid_mass_kg: f64,
) -> Result<Option<BPlaneEncounter>, DeflectionError> {
    let ok = beta.is_finite()
        && beta > 0.0
        && impactor_mass_kg.is_finite()
        && impactor_mass_kg >= 0.0
        && asteroid_mass_kg.is_finite()
        && asteroid_mass_kg > 0.0;
    if !ok {
        return Err(DeflectionError::InvalidInput(format!(
            "verify_cell needs β>0, m_sc≥0, M>0 (got β={beta}, m_sc={impactor_mass_kg}, M={asteroid_mass_kg})"
        )));
    }
    let dv = impact_impulse(metrics.v_rel_vec, beta, impactor_mass_kg, asteroid_mass_kg);
    deflection.evaluate(arrival_epoch, dv)
}

/// The outcome of solving for the impactor mass a launch window needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MassSolveOutcome {
    /// The target perigee is reachable; the impactor mass (kg) that just clears it.
    Feasible { impactor_mass_kg: f64 },
    /// No deliverable mass up to the cap clears the target along this geometry —
    /// the honest "this window can't do it" cell (the degenerate-direction case,
    /// and any window too weakly coupled to help). Carries the perigee reached at
    /// the cap so the caller can say *how close* it got.
    InfeasibleAtCap {
        /// Mass cap probed, kg.
        mass_cap_kg: f64,
        /// Perigee reached at the cap, metres.
        perigee_reached_m: f64,
    },
}

/// Solve for the impactor mass that raises the full-field b-plane perigee to
/// `target_perigee_m`, for one cell's coupled-direction impulse at
/// `arrival_epoch`. Doubles the mass from `seed_mass_kg` until the perigee clears
/// the target, then bisects; capped at `mass_cap_kg`.
///
/// The cap is the **degenerate-direction guard the advisor asked for from day
/// one**: when the arrival geometry projects poorly onto the track (`v_rel ⊥
/// v̂_ast`), no deliverable mass meaningfully deflects, and an uncapped bisection
/// would run away. Hitting the cap returns [`MassSolveOutcome::InfeasibleAtCap`]
/// — an honest window state, not a hang. Each probe is one full-field
/// propagation, so this is an on-demand operation, never a per-grid one.
#[allow(clippy::too_many_arguments)]
pub fn required_impactor_mass(
    deflection: &DeflectionScenario,
    arrival_epoch: Epoch,
    metrics: &TransferMetrics,
    beta: f64,
    asteroid_mass_kg: f64,
    target_perigee_m: f64,
    seed_mass_kg: f64,
    mass_cap_kg: f64,
) -> Result<MassSolveOutcome, DeflectionError> {
    let ok = beta.is_finite()
        && beta > 0.0
        && asteroid_mass_kg.is_finite()
        && asteroid_mass_kg > 0.0
        && target_perigee_m.is_finite()
        && target_perigee_m > 0.0
        && seed_mass_kg.is_finite()
        && seed_mass_kg > 0.0
        && mass_cap_kg.is_finite()
        && mass_cap_kg > seed_mass_kg;
    if !ok {
        return Err(DeflectionError::InvalidInput(format!(
            "required_impactor_mass needs β>0, M>0, target>0, 0<seed<cap (got β={beta}, M={asteroid_mass_kg}, target={target_perigee_m}, seed={seed_mass_kg}, cap={mass_cap_kg})"
        )));
    }

    let perigee_at = |mass: f64| -> Result<f64, DeflectionError> {
        let dv = impact_impulse(metrics.v_rel_vec, beta, mass, asteroid_mass_kg);
        // A clean miss (left the scan gate) is the best possible perigee → +∞;
        // a dead-centre bound pass reads as 0 (matches deflection's perigee scale).
        match deflection.evaluate(arrival_epoch, dv) {
            Ok(Some(bp)) => Ok(bp.perigee),
            Ok(None) => Ok(f64::INFINITY),
            Err(DeflectionError::Geometry(crate::geometry::GeometryError::NotHyperbolic {
                ..
            })) => Ok(0.0),
            Err(e) => Err(e),
        }
    };

    // Already clear at the seed? Then any mass ≥ seed works; report the seed.
    if perigee_at(seed_mass_kg)? >= target_perigee_m {
        return Ok(MassSolveOutcome::Feasible {
            impactor_mass_kg: seed_mass_kg,
        });
    }

    // Grow the mass to bracket the crossing, capped.
    let mut lo = seed_mass_kg;
    let mut hi = seed_mass_kg * 2.0;
    loop {
        if hi >= mass_cap_kg {
            let reached = perigee_at(mass_cap_kg)?;
            if reached < target_perigee_m {
                return Ok(MassSolveOutcome::InfeasibleAtCap {
                    mass_cap_kg,
                    perigee_reached_m: reached,
                });
            }
            hi = mass_cap_kg;
            break;
        }
        if perigee_at(hi)? >= target_perigee_m {
            break;
        }
        lo = hi;
        hi *= 2.0;
    }

    // Bisect the mass bracket [lo, hi] to a tight relative width.
    for _ in 0..80 {
        if (hi - lo) <= 1e-4 * hi {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if perigee_at(mid)? >= target_perigee_m {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Ok(MassSolveOutcome::Feasible {
        impactor_mass_kg: hi,
    })
}

/// Convenience: the SSB vector impulse a kinetic impactor imparts for one cell,
/// exposed so a frontend can show the actual Δv it is about to apply.
pub fn cell_impulse(
    metrics: &TransferMetrics,
    beta: f64,
    impactor_mass_kg: f64,
    asteroid_mass_kg: f64,
) -> Vector3<f64> {
    impact_impulse(metrics.v_rel_vec, beta, impactor_mass_kg, asteroid_mass_kg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::launch_vehicle::{ATLAS_V_551, FALCON_HEAVY_EXPENDABLE};

    const MU_SUN: f64 = 1.327_124_400_18e20;
    const AU: f64 = 1.495_978_707e11;

    // --- Pure metrics (kernel-free) -----------------------------------------

    #[test]
    fn impulse_is_along_v_rel_and_scales_with_mass_ratio() {
        let v_rel = Vector3::new(3000.0, -1000.0, 500.0);
        let beta = 3.6;
        let m_sc = 600.0;
        let m_ast = 2.0e10;
        let dv = impact_impulse(v_rel, beta, m_sc, m_ast);
        // Direction: parallel to v_rel.
        let cross = dv.cross(&v_rel).norm();
        assert!(cross < 1e-6, "impulse not parallel to v_rel");
        // Magnitude: β·(m/M)·|v_rel|.
        let expected = beta * (m_sc / m_ast) * v_rel.norm();
        assert!((dv.norm() - expected).abs() / expected < 1e-12);
    }

    #[test]
    fn arrival_on_the_transfer_gives_zero_relative_velocity() {
        // If the asteroid's velocity at arrival *is* the Lambert arrival velocity,
        // the impactor arrives with zero relative velocity — a clean identity that
        // pins v_rel = v2 − v_ast and the C3 = |v1 − v⊕|² wiring/units.
        let earth = StateVector::new(
            Vector3::new(AU, 0.0, 0.0),
            Vector3::new(0.0, 29_780.0, 0.0),
        );
        // Pick an asteroid position and a time of flight, solve Lambert to learn
        // the arrival velocity, then *define* the asteroid's velocity to match it.
        let r2 = Vector3::new(-0.2 * AU, 1.3 * AU, 0.05 * AU);
        let tof = 0.35 * 365.25 * 86400.0;
        let sol = lambert_universal(earth.position, r2, tof, MU_SUN, true).unwrap();
        let asteroid = StateVector::new(r2, sol.v2);

        let m = transfer_metrics(earth, asteroid, tof, MU_SUN, true)
            .unwrap()
            .unwrap();
        assert!(
            m.arrival_v_rel_ms < 1e-6,
            "v_rel should vanish, got {}",
            m.arrival_v_rel_ms
        );
        assert!(m.along_track_proj_ms.abs() < 1e-6);
        // C3 must equal |v1 − v⊕|² in km²/s² — an independent recompute of the units.
        let c3_expect = (sol.v1 - earth.velocity).norm_squared() * 1e-6;
        assert!((m.c3_km2_s2 - c3_expect).abs() / c3_expect < 1e-12);
    }

    #[test]
    fn head_on_arrival_projects_negatively_on_the_track() {
        // Construct an arrival whose relative velocity opposes the asteroid's
        // motion: the along-track projection must be negative (a retrograde,
        // orbit-shrinking push), and |proj| ≤ |v_rel|.
        let earth = StateVector::new(
            Vector3::new(AU, 0.0, 0.0),
            Vector3::new(0.0, 29_780.0, 0.0),
        );
        let r2 = Vector3::new(0.1 * AU, 1.25 * AU, 0.0);
        let tof = 0.3 * 365.25 * 86400.0;
        let sol = lambert_universal(earth.position, r2, tof, MU_SUN, true).unwrap();
        // Asteroid moving fast prograde-ish, so the transfer (slower at arrival)
        // meets it with a component opposing its velocity.
        let asteroid = StateVector::new(r2, sol.v2 * 1.4);
        let m = transfer_metrics(earth, asteroid, tof, MU_SUN, true)
            .unwrap()
            .unwrap();
        assert!(m.along_track_proj_ms < 0.0, "expected retrograde projection");
        assert!(m.along_track_proj_ms.abs() <= m.arrival_v_rel_ms + 1e-6);
    }

    #[test]
    fn no_transfer_when_arrival_precedes_launch() {
        // A non-positive time of flight is a grid gap, surfaced as InvalidInput
        // from Lambert and mapped to Ok(None) here would be wrong — transfer_metrics
        // forwards the InvalidInput; the grid guards tof ≥ min_tof upstream. Assert
        // the Lambert InvalidInput propagates so the grid's guard is load-bearing.
        let earth = StateVector::new(Vector3::new(AU, 0.0, 0.0), Vector3::new(0.0, 29_780.0, 0.0));
        let ast = StateVector::new(Vector3::new(0.0, AU, 0.0), Vector3::new(-29_780.0, 0.0, 0.0));
        assert!(transfer_metrics(earth, ast, 0.0, MU_SUN, true).is_err());
    }

    #[test]
    fn cell_delivery_maps_c3_to_the_vehicle_table() {
        // A cell at a Mars-class C3 delivers the vehicle's tabulated payload, and
        // the along-track Δv is β·(m/M)·proj on that mass.
        let metrics = TransferMetrics {
            c3_km2_s2: 15.0,
            arrival_v_rel_ms: 6000.0,
            along_track_proj_ms: 4000.0,
            v_rel_vec: Vector3::new(4000.0, 3000.0, 1000.0),
        };
        let m_ast = 2.0e10;
        let beta = 3.6;
        let d = cell_delivery(&metrics, &FALCON_HEAVY_EXPENDABLE, beta, m_ast);
        assert!(d.feasible);
        let expected_mass = FALCON_HEAVY_EXPENDABLE.payload_kg(15.0);
        assert!((d.payload_kg - expected_mass).abs() < 1e-6);
        let expected_dv = beta * (expected_mass / m_ast) * 4000.0;
        assert!((d.along_track_dv_ms - expected_dv).abs() / expected_dv < 1e-12);
    }

    #[test]
    fn cell_delivery_infeasible_above_vehicle_max_c3() {
        // Beyond Atlas V 551's tabulated C3, no mass is delivered — the cell is
        // honestly infeasible, not extrapolated.
        let metrics = TransferMetrics {
            c3_km2_s2: ATLAS_V_551.max_c3_km2_s2() + 10.0,
            arrival_v_rel_ms: 5000.0,
            along_track_proj_ms: 3000.0,
            v_rel_vec: Vector3::new(3000.0, 0.0, 0.0),
        };
        let d = cell_delivery(&metrics, &ATLAS_V_551, 3.6, 2.0e10);
        assert!(!d.feasible);
        assert_eq!(d.payload_kg, 0.0);
        assert_eq!(d.along_track_dv_ms, 0.0);
    }

    // --- Kernel-gated: the grid + on-demand verify over a real scenario ------

    #[test]
    fn porkchop_grid_and_verify_over_the_real_field() {
        use crate::scenario::{ImpactorConfig, RealFieldScenario};
        if crate::kernels::resolve_for_test("porkchop_grid_and_verify_over_the_real_field").is_none()
        {
            return;
        }
        let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");

        // A small launch/arrival grid inside the campaign: launch in the first
        // portion, arrive with enough lead before impact.
        let t0 = sc.epoch0().tdb_seconds_past_j2000();
        let t_impact = sc.impact_epoch().tdb_seconds_past_j2000();
        let span = t_impact - t0;
        let day = 86_400.0;

        let launch_epochs: Vec<Epoch> = (0..5)
            .map(|i| Epoch::from_tdb_seconds_past_j2000(t0 + 0.05 * span + (i as f64) * 30.0 * day))
            .collect();
        let arrival_epochs: Vec<Epoch> = (0..5)
            .map(|j| {
                Epoch::from_tdb_seconds_past_j2000(t0 + 0.35 * span + (j as f64) * 40.0 * day)
            })
            .collect();

        let pork = porkchop_grid(&sc, &launch_epochs, &arrival_epochs, 30.0 * day, true)
            .expect("grid builds");

        // At least one real transfer, and every C3 finite and non-negative.
        let mut feasible: Option<(usize, usize, TransferMetrics)> = None;
        for (i, row) in pork.cells.iter().enumerate() {
            for (j, cell) in row.iter().enumerate() {
                if let PorkchopCell::Transfer(m) = cell {
                    assert!(m.c3_km2_s2.is_finite() && m.c3_km2_s2 >= 0.0);
                    assert!(m.arrival_v_rel_ms.is_finite() && m.arrival_v_rel_ms >= 0.0);
                    if feasible.is_none() {
                        feasible = Some((i, j, *m));
                    }
                }
            }
        }
        let (_, j, metrics) = feasible.expect("grid should contain at least one transfer");

        // Deliver a Falcon-Heavy-class impactor and verify the coupled full-field
        // encounter runs and returns a finite perigee (or a clean miss).
        let m_ast = 2.0e10; // ~sub-km rock
        let delivery = cell_delivery(&metrics, &FALCON_HEAVY_EXPENDABLE, 3.6, m_ast);
        let m_sc = delivery.payload_kg.max(1.0);

        let ds = sc.deflection().expect("deflection scenario");
        let outcome = verify_cell(&ds, arrival_epochs[j], &metrics, 3.6, m_sc, m_ast)
            .expect("verify runs");
        if let Some(bp) = outcome {
            assert!(bp.perigee.is_finite() && bp.perigee >= 0.0);
        }
        // Either a finite perigee or a clean miss (None) is acceptable; the point
        // is the on-demand full-field path composes end to end.
    }
}
