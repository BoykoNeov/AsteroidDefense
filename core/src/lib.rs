//! `asteroid_core` — headless, deterministic astrodynamics core.
//!
//! Single source of truth for the simulation. **No renderer / UI dependency
//! ever links here** (HANDOFF §10 invariant). This crate provides the ephemeris
//! loader, the core physics types — [`Epoch`](epoch::Epoch),
//! [`StateVector`](state::StateVector), and
//! [`OrbitalElements`](elements::OrbitalElements) with the element↔state map
//! (§10.3) — the analytic Kepler [`Propagator`](propagator::Propagator) (§10.4),
//! and the composable [`ForceModel`](forces::ForceModel) + swappable
//! [`Integrator`](integrator::Integrator) (§10.7): fixed-step [`Rk4`](integrator::Rk4)
//! and the adaptive [`Dop853`](integrator::Dop853) MVP encounter integrator, and
//! the [`geometry`] b-plane hit test (§10.8) that turns a close approach into a
//! hit/miss answer. The [`clock`] (§10.9) samples the [`Dop853`](integrator::Dop853)
//! dense output at a fixed cadence, serving sub-snapshot queries from the 7th-order
//! continuous extension rather than linear interpolation. The [`close_approach`]
//! detector (§10.9) root-finds the range-rate on that same continuous trajectory to
//! locate geocentric closest approach and feed the Earth-relative state into the
//! b-plane geometry — closing the encounter pipeline into a hit/miss answer.

pub mod clock;
pub mod close_approach;
pub mod deflection;
pub mod elements;
pub mod ephemeris;
pub mod epoch;
pub mod forces;
pub mod geometry;
pub mod horizons;
pub mod integrator;
pub mod kernels;
pub mod perturber_field;
pub mod propagator;
pub mod scenario;
pub mod state;

pub use clock::{Clock, ClockError};
pub use close_approach::{
    closest_approach, find_close_approaches, CloseApproach, CloseApproachError, GeocentricState,
    ScanOptions,
};
pub use deflection::{
    along_track_unit, apply_impulse, kinetic_impactor_dv, DeflectionError, DeflectionScenario,
    DvSolveTol,
};
pub use elements::{ElementsError, OrbitalElements};
pub use horizons::{Neo, NeoError};
pub use epoch::Epoch;
pub use forces::relativity::{CentralBodyState, FixedCentralBody, Relativity1PN, SPEED_OF_LIGHT_M_S};
pub use forces::yarkovsky::YarkovskyA2;
pub use forces::{CompositeForce, ForceError, ForceModel};
pub use geometry::{
    BPlaneEncounter, GeometryError, EARTH_EQUATORIAL_RADIUS_M, EARTH_MEAN_RADIUS_M,
};
pub use integrator::{propagate_fixed, DenseSegment, Dop853, Integrator, IntegratorError, Rk4};
pub use perturber_field::{tier1_perturber_field, EphemerisPerturber, TIER1_PERTURBER_FRAMES};
pub use propagator::{KeplerPropagator, Propagator, PropagatorError};
pub use scenario::{
    DeflectedArc, EncounterFrame, ImpactorConfig, RealFieldScenario, ScenarioError, SweepPoint,
    ENCOUNTER_HALF_WINDOW_SECONDS, ENCOUNTER_SAMPLES,
};
pub use state::StateVector;

/// Crate version string, surfaced so the viewer/validation layers can report
/// which core build produced a result (determinism = same-build-same-output).
pub const CORE_VERSION: &str = env!("CARGO_PKG_VERSION");
