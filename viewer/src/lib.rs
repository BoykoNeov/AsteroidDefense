//! `viewer` — the pure-Rust (egui) frontend for the MVP (HANDOFF §10 task 10).
//!
//! The renderer is deliberately thin: all physics lives in [`asteroid_core`],
//! and the *mission* framing — a designer Earth-impactor over the real DE440
//! field and the Δv-vs-lead-time sweep across it — lives in [`scenario`], which
//! is **egui-free** so it can be exercised headlessly (the `curve` binary,
//! Commit B1) before any GPU stack links. The egui app (`main.rs`, Commit B2)
//! renders the same [`scenario`] data: the headline curve and the
//! rewind→nudge→re-propagate animation.

pub mod scenario;
