//! Measure the per-cell cost of the porkchop grid's transfer selection.
//!
//! `best_transfer_metrics` at `max_revolutions = 1` runs the direct Newton solve
//! plus two bracket-and-bisect multi-rev solves (a few hundred `tof_from_z`
//! evaluations each), where the old grid ran one Newton. The Godot heatmap view is
//! the next task and this project has twice been bitten by an unmeasured per-frame
//! cost, so the number goes on the record before that view is built.
//!
//! Kernel-free: the selection is pure scalar math over two states, so fabricated
//! endpoints measure the same code the grid runs.
//!
//!   cargo run -p asteroid_core --release --example bench_porkchop_cell

use asteroid_core::mission::best_transfer_metrics;
use asteroid_core::StateVector;
use nalgebra::Vector3;
use std::time::Instant;

const AU: f64 = 1.495_978_707e11;
const MU_SUN: f64 = 1.327_124_400_18e20;
const YEAR: f64 = 365.25 * 86400.0;

fn main() {
    // A spread of geometries and times of flight, so the timing averages over
    // cells that solve, cells that gap, and cells where the multi-rev band is
    // empty — the mix a real grid contains.
    let earth = StateVector::new(
        Vector3::new(AU, 0.0, 0.0),
        Vector3::new(0.0, 29_780.0, 0.0),
    );

    for (label, n) in [("100x100", 100usize), ("200x200", 200usize)] {
        for max_rev in [0u32, 1, 2] {
            let start = Instant::now();
            let mut solved = 0usize;
            for i in 0..n {
                for j in 0..n {
                    // Sweep the target around its orbit and the time of flight
                    // across the range a multi-year campaign grid spans.
                    let theta = (j as f64) / (n as f64) * std::f64::consts::TAU;
                    let r = 1.2 * AU;
                    let ast = StateVector::new(
                        Vector3::new(r * theta.cos(), r * theta.sin(), 0.05 * AU),
                        Vector3::new(-22_000.0 * theta.sin(), 22_000.0 * theta.cos(), 500.0),
                    );
                    let tof = (0.3 + 3.2 * (i as f64) / (n as f64)) * YEAR;
                    if let Ok(Some(_)) =
                        best_transfer_metrics(earth, ast, tof, MU_SUN, true, max_rev)
                    {
                        solved += 1;
                    }
                }
            }
            let elapsed = start.elapsed();
            let cells = n * n;
            println!(
                "{label} max_rev={max_rev}: {:>8.1} ms total, {:>7.1} us/cell, {solved}/{cells} solved",
                elapsed.as_secs_f64() * 1e3,
                elapsed.as_secs_f64() * 1e6 / cells as f64,
            );
        }
    }
}
