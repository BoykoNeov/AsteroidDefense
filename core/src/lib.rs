//! `asteroid_core` — headless, deterministic astrodynamics core.
//!
//! Single source of truth for the simulation. **No renderer / UI dependency
//! ever links here** (HANDOFF §10 invariant). The force model and the
//! encounter/b-plane logic land in later tasks (§10.7–8); this crate currently
//! provides the ephemeris loader, the core physics types —
//! [`Epoch`](epoch::Epoch), [`StateVector`](state::StateVector), and
//! [`OrbitalElements`](elements::OrbitalElements) with the element↔state map
//! (§10.3) — and the analytic Kepler [`Propagator`](propagator::Propagator)
//! (§10.4).

pub mod elements;
pub mod ephemeris;
pub mod epoch;
pub mod propagator;
pub mod state;

pub use elements::{ElementsError, OrbitalElements};
pub use epoch::Epoch;
pub use propagator::{KeplerPropagator, Propagator, PropagatorError};
pub use state::StateVector;

/// Crate version string, surfaced so the viewer/validation layers can report
/// which core build produced a result (determinism = same-build-same-output).
pub const CORE_VERSION: &str = env!("CARGO_PKG_VERSION");
