//! `asteroid_core` — headless, deterministic astrodynamics core.
//!
//! Single source of truth for the simulation. **No renderer / UI dependency
//! ever links here** (HANDOFF §10 invariant). This crate provides the ephemeris
//! loader, the core physics types — [`Epoch`](epoch::Epoch),
//! [`StateVector`](state::StateVector), and
//! [`OrbitalElements`](elements::OrbitalElements) with the element↔state map
//! (§10.3) — the analytic Kepler [`Propagator`](propagator::Propagator) (§10.4),
//! and the composable [`ForceModel`](forces::ForceModel) + swappable
//! [`Integrator`](integrator::Integrator) (§10.7, RK4 first). The b-plane /
//! encounter logic and the dop853 encounter integrator land in later tasks.

pub mod elements;
pub mod ephemeris;
pub mod epoch;
pub mod forces;
pub mod integrator;
pub mod propagator;
pub mod state;

pub use elements::{ElementsError, OrbitalElements};
pub use epoch::Epoch;
pub use forces::{CompositeForce, ForceError, ForceModel};
pub use integrator::{propagate_fixed, Integrator, IntegratorError, Rk4};
pub use propagator::{KeplerPropagator, Propagator, PropagatorError};
pub use state::StateVector;

/// Crate version string, surfaced so the viewer/validation layers can report
/// which core build produced a result (determinism = same-build-same-output).
pub const CORE_VERSION: &str = env!("CARGO_PKG_VERSION");
