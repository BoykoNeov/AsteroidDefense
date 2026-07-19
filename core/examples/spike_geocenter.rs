//! Task-0.5 de-risk spike (pillar b) — runnable report.
//!
//! Proves the ANISE DE-position reader returns a *reconstructed geocenter*
//! (Earth 399), not the Earth–Moon barycenter (EMB, 3), for a known epoch —
//! the HANDOFF §5 footgun that would otherwise inject a ~4671 km,
//! Earth-radius-scale b-plane error.
//!
//! Usage:
//!   cargo run -p asteroid_core --example spike_geocenter -- <path/to/de440s.bsp>
//! or set ASTEROID_DE_KERNEL and pass no argument.

use anise::prelude::Epoch;
use anise::time::TimeScale;
use asteroid_core::ephemeris::{verify_geocenter_reconstruction, Ephemeris};

fn main() {
    let kernel = std::env::args()
        .nth(1)
        .or_else(|| asteroid_core::kernels::resolve().map(|k| k.bsp.display().to_string()))
        .unwrap_or_else(|| {
            eprintln!("usage: spike_geocenter <de440s.bsp>  (or set ASTEROID_DE_KERNEL)");
            std::process::exit(2);
        });

    let eph = match Ephemeris::load(&kernel) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("FAILED to load kernel: {e}");
            std::process::exit(1);
        }
    };
    println!("Loaded kernel: {}", eph.kernel_path().display());

    // Sample a few epochs across the DE span so the ~4671 km offset is shown to
    // track the real (varying) Earth–Moon distance, not a single lucky value.
    let epochs = [
        Epoch::from_gregorian(1900, 1, 1, 0, 0, 0, 0, TimeScale::TDB),
        Epoch::from_gregorian(2000, 1, 1, 12, 0, 0, 0, TimeScale::TDB),
        Epoch::from_gregorian(2020, 1, 1, 0, 0, 0, 0, TimeScale::TDB),
        Epoch::from_gregorian(2035, 7, 4, 6, 0, 0, 0, TimeScale::TDB),
    ];

    let mut all_pass = true;
    for epoch in epochs {
        match verify_geocenter_reconstruction(&eph, epoch) {
            Ok(c) => {
                let ok = c.passes();
                all_pass &= ok;
                println!("\n=== {epoch} ===");
                println!(
                    "  geocenter (Earth 399) vs SSB : [{:.3}, {:.3}, {:.3}] km",
                    c.geocenter_ssb_km.x, c.geocenter_ssb_km.y, c.geocenter_ssb_km.z
                );
                println!(
                    "  EMB (3) vs SSB               : [{:.3}, {:.3}, {:.3}] km",
                    c.emb_ssb_km.x, c.emb_ssb_km.y, c.emb_ssb_km.z
                );
                println!(
                    "  |geocenter - EMB|            : {:.3} km   (expect ~4671; must be != 0)",
                    c.geocenter_emb_offset_km
                );
                println!(
                    "  Earth-Moon distance          : {:.1} km   (expect 356k-406k)",
                    c.earth_moon_distance_km
                );
                println!(
                    "  EMRAT self-consistency residual: {:.6} km   (expect < 1)",
                    c.emrat_residual_km
                );
                println!("  -> {}", if ok { "PASS" } else { "FAIL" });
            }
            Err(e) => {
                all_pass = false;
                println!("\n=== {epoch} ===\n  ERROR: {e}");
            }
        }
    }

    println!(
        "\nSPIKE PILLAR B (ANISE geocenter reconstruction): {}",
        if all_pass { "PASS" } else { "FAIL" }
    );
    std::process::exit(if all_pass { 0 } else { 1 });
}
