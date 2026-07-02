//! `validation` — the oracle-ladder test harness.
//!
//! Asserts the `asteroid_core` propagators/force model against reference states
//! committed as JSON fixtures from the offline `pyref/` pipeline (hapsira,
//! REBOUND, ASSIST). The oracle ladder (HANDOFF §6):
//! - **Rung 1 — free-invariant** property tests (no external oracle needed),
//!   `tests/free_invariants.rs` (§10.5).
//! - **Rung 2 — analytic two-body** vs hapsira, `tests/kepler_reference.rs`
//!   loading `fixtures/kepler_two_body.json` (§10.6).
//!
//! Higher rungs (numerical integrator vs REBOUND/ASSIST) land with the force
//! model in §10.7. This `lib` itself only confirms the crate links `core`; the
//! fixtures are `include_str!`'d directly by the integration tests.

/// Marker that the validation crate is wired to the core it validates.
pub fn validates_core() -> &'static str {
    asteroid_core::CORE_VERSION
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn links_against_core() {
        assert!(!validates_core().is_empty());
    }
}
