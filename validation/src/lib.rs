//! `validation` — the oracle-ladder test harness.
//!
//! Asserts the `asteroid_core` propagators/force model against reference states
//! committed as JSON fixtures from the offline `pyref/` pipeline (hapsira,
//! REBOUND, ASSIST). Real fixtures + the free-invariant property tests land in
//! HANDOFF §10 tasks 5–7; this scaffold only confirms the crate links `core`
//! and serde.

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
