//! `validation` — the oracle-ladder test harness.
//!
//! Asserts the `asteroid_core` propagators/force model against reference states
//! committed as JSON fixtures from the offline `pyref/` pipeline (hapsira,
//! REBOUND, ASSIST). The **free-invariant** property tests (rung 1 of the §6
//! oracle ladder — no external oracle needed) live in `tests/free_invariants.rs`
//! (HANDOFF §10.5). The JSON fixtures + their loaders land in §10 tasks 6–7; this
//! `lib` only confirms the crate links `core` and serde.

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
