//! `asteroid_core` — headless, deterministic astrodynamics core.
//!
//! Single source of truth for the simulation. **No renderer / UI dependency
//! ever links here** (HANDOFF §10 invariant). Physics types, propagators, the
//! force model, and the encounter/b-plane logic land in later tasks (§10.3–8);
//! this scaffold only stands up the crate and the ephemeris loader entry point.

pub mod ephemeris;

/// Crate version string, surfaced so the viewer/validation layers can report
/// which core build produced a result (determinism = same-build-same-output).
pub const CORE_VERSION: &str = env!("CARGO_PKG_VERSION");
