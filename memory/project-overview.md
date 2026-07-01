---
name: project-overview
description: "What the AsteroidDefense project is, where the spec lives, and current phase"
metadata: 
  node_type: memory
  type: project
  originSessionId: f5cc34dd-dad9-418c-9791-57031e47c59c
---

**Asteroid Deflection Simulator** — an educational planetary-defense sim whose
single thesis is: *deflecting an asteroid early is dramatically more effective
than late*; the headline screen is a Δv-vs-lead-time curve. Realism is a primary
stated goal. Headless deterministic **Rust core** = single source of truth;
**egui** pure-Rust viewer for the MVP; **Godot** (gdext) in Phase 2.

The full **locked spec** lives in `HANDOFF.md` (architecture, tiered force model,
validation oracle ladder, hard problems, task-by-task plan). `README.md` is the
public summary. Don't re-litigate decisions marked locked there.

**License:** Boyko Non-Commercial License v1.0 (BNCL-1.0) — proprietary,
non-commercial use permitted, commercial use requires separate written
permission. Relicensed from Apache-2.0 on 2026-07-01 (commit bab4853); it is no
longer OSI open source. **Public GitHub repo:** owner `BoykoNeov` (created
2026-06-23). Only hifitime/ANISE (MPL) + nalgebra (Apache/MIT) link into the
shipped binary; consuming those permissive/MPL crates stays compatible under
BNCL. GPL/AGPL oracles (REBOUND, ASSIST, GRSS, nyx) stay offline in
`pyref/` — never in any Cargo.toml.

**Current phase (as of 2026-07-01):** §10 task 1 DONE — Cargo workspace
scaffolded (`core/` lib = `asteroid_core`, package renamed to dodge the std
`core` shadowing trap; `viewer/` egui bin; `validation/` lib; `pyref/` stays a
non-member Python dir). `resolver = "2"`, edition 2021, `cargo build` green,
`core`'s dep tree has zero egui/eframe/wgpu (the §10 no-UI-in-core invariant,
enforced from day one). ANISE loader is an intentional path-taking **stub** in
`core/src/ephemeris.rs` — real DE-position/geocenter wiring is task 2. Deps:
anise 0.10.3, hifitime 4.3.0, nalgebra 0.35.0 (anise pulled network features
by default — trim to `default-features=false` when task 2 reveals the loader's
real needs). **Next concrete step = §10 task 2**, the task-0.5 de-risk spike:
confirm ASSIST+DE441 builds offline and the ANISE DE-position reader returns a
real reconstructed geocenter (not the EMB), with the written
fallback-to-Option-B trigger. See [[git-workflow]] for the commit/push cadence.
