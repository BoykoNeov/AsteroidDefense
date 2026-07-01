//! `viewer` — the pure-Rust (egui) frontend for the MVP.
//!
//! The headline Δv-vs-lead-time chart and the rewind→nudge→re-propagate
//! animation live here (HANDOFF §10 task 10). This scaffold only confirms the
//! crate wires against `asteroid_core` with zero physics in the UI layer.

fn main() {
    println!(
        "Asteroid Deflection Simulator — viewer scaffold (core {})",
        asteroid_core::CORE_VERSION
    );
}
