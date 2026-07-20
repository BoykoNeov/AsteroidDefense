//! Probe a JPL Horizons per-object small-body SPK: does ANISE actually *read* it?
//!
//! This is the gate the Horizons-NEO work hangs on, and it is not rhetorical.
//! `sb441-n16.bsp` — the small-body kernel already mounted — is **SPK type 2**
//! (Chebyshev position). A Horizons-generated per-object SPK is **type 21**
//! (extended difference lines), a different segment format entirely. "Same read
//! path" is a claim about the call site, not about the decoder underneath it, so
//! this probe evaluates a real position rather than trusting that ANISE covers 21.
//!
//! It also prints the NAIF id the kernel *answers to*. Horizons uses the extended
//! small-body numbering (`20000000 + number`, so 99942 Apophis is **20099942**),
//! not the `2000000 + number` convention `sb441` uses for the numbered
//! perturbers — near-identical, and hardcoding the wrong one from recall yields a
//! lookup that fails for a reason that looks like anything but a typo.
//!
//! Usage:
//!   cargo run -p asteroid_core --example probe_horizons -- <neo.bsp> <naif_id>
//! The DE pair comes from `ASTEROID_DE_KERNEL` / `ASTEROID_PLANETARY_CONSTANTS`
//! or the conventional directories (`kernels::resolve`).

use anise::constants::frames::SUN_J2000;
use anise::prelude::{Epoch as AniseEpoch, Frame};
use anise::time::TimeScale;
use asteroid_core::ephemeris::Ephemeris;

/// One AU in km, for a sanity read on the printed distances.
const AU_KM: f64 = 149_597_870.7;

fn main() {
    let mut args = std::env::args().skip(1);
    let (Some(neo), Some(id)) = (args.next(), args.next()) else {
        eprintln!("usage: probe_horizons <neo.bsp> <naif_id>");
        std::process::exit(2);
    };
    let id: i32 = id.parse().unwrap_or_else(|_| {
        eprintln!("naif_id must be an integer");
        std::process::exit(2);
    });

    let Some(k) = asteroid_core::kernels::resolve() else {
        eprintln!("{}", asteroid_core::kernels::not_found_message());
        std::process::exit(1);
    };
    let (bsp, pca) = k.as_strs();
    println!("DE pair    : {bsp}\n             {pca}  ({})", k.source);
    println!("NEO kernel : {neo}");
    println!("NAIF id    : {id}\n");

    let eph = match Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .and_then(|e| e.with_constants(&neo))
    {
        Ok(e) => e,
        Err(e) => {
            println!("MOUNT FAILED: {e}");
            println!("\nPROBE (Horizons SPK readable): FAIL");
            std::process::exit(1);
        }
    };
    println!("mounted OK\n");

    // Spread the samples across a couple of centuries so a narrow-span kernel
    // shows its edges instead of passing on one lucky epoch in the middle.
    let years = [1960, 2000, 2029, 2031, 2043, 2100, 2140];
    let frame = Frame::from_ephem_j2000(id);
    let mut any = false;
    println!("{:<8} {:>18} {:>14}", "year", "|r_helio| (km)", "AU");
    println!("{}", "-".repeat(42));
    for y in years {
        let epoch = AniseEpoch::from_gregorian(y, 1, 1, 0, 0, 0, 0, TimeScale::TDB);
        match eph.position_km(frame, SUN_J2000, epoch) {
            Ok(p) => {
                any = true;
                println!("{:<8} {:>18.1} {:>14.4}", y, p.norm(), p.norm() / AU_KM);
            }
            Err(e) => println!("{y:<8} {e}"),
        }
    }

    println!(
        "\nPROBE (Horizons SPK readable): {}",
        if any { "PASS" } else { "FAIL" }
    );
    std::process::exit(if any { 0 } else { 1 });
}
