//! Mission scenario re-exports + the on-disk curve cache format.
//!
//! The physics — the designer impactor, the Δv sweep, the encounter frame — now
//! lives in [`asteroid_core::scenario`], so the egui viewer and the Godot gdext
//! binding drive one validated scenario (one source of truth for the drawn
//! tracks and the headline numbers). This module re-exports those types and adds
//! the viewer-only, **serde**-backed curve cache (`curve.json`): the `curve`
//! binary writes it once (the ~680 s sweep is a fixed property of the design),
//! and the egui app loads it instantly instead of recomputing on startup.

use serde::{Deserialize, Serialize};

pub use asteroid_core::scenario::{
    EncounterFrame, ImpactorConfig, RealFieldScenario, ScenarioError, SweepPoint,
    ENCOUNTER_HALF_WINDOW_SECONDS, ENCOUNTER_SAMPLES,
};

/// One point of the headline curve, serialisable for the `curve.json` cache —
/// the disk form of a core [`SweepPoint`] (core stays serde-free).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CurvePoint {
    /// Lead time before impact, seconds.
    pub lead_seconds: f64,
    /// Lead time expressed in heliocentric orbital periods.
    pub lead_periods: f64,
    /// Minimum along-track Δv to clear the target perigee, m/s.
    pub required_dv: f64,
}

impl From<SweepPoint> for CurvePoint {
    fn from(p: SweepPoint) -> Self {
        Self {
            lead_seconds: p.lead_seconds,
            lead_periods: p.lead_periods,
            required_dv: p.required_dv,
        }
    }
}

/// The default filename the `curve` binary writes and the egui app reads, in the
/// current working directory. The curve is a fixed property of the designed
/// scenario, so it is computed once (the ~680 s sweep) and cached to disk — the
/// app loads it instantly and never recomputes (there is no config-editing UI).
pub const DEFAULT_CURVE_JSON: &str = "curve.json";

/// The serialised headline curve plus the scenario summary it was computed for,
/// so the egui app can label and scale the plot without rebuilding the scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CurveFile {
    /// Heliocentric semi-major axis of the designed impactor, m.
    pub semi_major_axis_m: f64,
    /// Heliocentric orbital period, seconds — the x-axis unit for the curve.
    pub period_seconds: f64,
    /// Lead available at the campaign start (`impact_epoch − epoch0`), seconds —
    /// the ceiling the sweep clamps over-long leads to.
    pub max_lead_seconds: f64,
    /// Target b-plane perigee the sweep solved each point to, m.
    pub target_perigee_m: f64,
    /// The swept curve points, ascending in lead.
    pub points: Vec<CurvePoint>,
}
