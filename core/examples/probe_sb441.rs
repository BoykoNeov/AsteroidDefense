//! Probe the 16 sb441 asteroid perturbers: does ANISE resolve a POSITION and a GM
//! for each, once the small-body kernel is chained onto the DE pair?
//!
//! The Tier-1 field pairs each body's position with a GM ANISE resolves for the
//! *same* frame. Enrolling the 16 sb441 bodies as force perturbers needs that same
//! pairing to hold for NAIF ids `2000000 + number`. sb441-n16.bsp is a bare SPK
//! (positions); ASSIST carries the 16 masses in its *own* C table, not the kernel,
//! so the open question this probe settles empirically is whether `gm_km3_s2`
//! resolves for these frames from the shipped constants — or whether the GMs must
//! be supplied from a hardcoded DE441 table.
//!
//! Usage: set ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS (the sb441 kernel
//! is discovered alongside the DE kernel) and run:
//!   cargo run -p asteroid_core --example probe_sb441

use anise::constants::frames::SSB_J2000;
use anise::prelude::{Epoch as AniseEpoch, Frame};
use anise::time::TimeScale;
use asteroid_core::ephemeris::Ephemeris;

/// The 16 sb441 bodies by NAIF id (2000000 + number) and name.
const SB441: &[(i32, &str)] = &[
    (2000001, "Ceres"),
    (2000002, "Pallas"),
    (2000003, "Juno"),
    (2000004, "Vesta"),
    (2000007, "Iris"),
    (2000010, "Hygiea"),
    (2000015, "Eunomia"),
    (2000016, "Psyche"),
    (2000031, "Euphrosyne"),
    (2000052, "Europa"),
    (2000065, "Cybele"),
    (2000087, "Sylvia"),
    (2000088, "Thisbe"),
    (2000107, "Camilla"),
    (2000511, "Davida"),
    (2000704, "Interamnia"),
];

fn main() {
    let Some(k) = asteroid_core::kernels::resolve() else {
        eprintln!("no DE kernel pair resolved");
        std::process::exit(2);
    };
    let Some(sb) = k.small_bodies.clone() else {
        eprintln!("no sb441 small-body kernel found next to the DE kernel");
        std::process::exit(2);
    };

    let eph = Ephemeris::load(&k.bsp)
        .and_then(|e| e.with_constants(&k.pca))
        .and_then(|e| e.with_constants(&sb))
        .unwrap_or_else(|e| {
            eprintln!("FAILED to load kernels: {e}");
            std::process::exit(1);
        });
    println!("DE kernel : {}", k.bsp.display());
    println!("constants : {}", k.pca.display());
    println!("small bod : {}\n", sb.display());

    let epoch = AniseEpoch::from_gregorian(2020, 1, 1, 0, 0, 0, 0, TimeScale::TDB);
    println!("{:<22} {:>16} {:>20} {:>8}", "body", "|r_SSB| (km)", "μ (km³/s²)", "status");
    println!("{}", "-".repeat(70));

    let mut pos_ok = 0;
    let mut gm_ok = 0;
    for &(id, name) in SB441 {
        let frame = Frame::from_ephem_j2000(id);
        let pos = eph.position_km(frame, SSB_J2000, epoch);
        let gm = eph.gm_km3_s2(frame);
        let pos_s = match &pos {
            Ok(p) => {
                pos_ok += 1;
                format!("{:.1}", p.norm())
            }
            Err(e) => format!("ERR: {e}"),
        };
        let gm_s = match &gm {
            Ok(g) => {
                gm_ok += 1;
                format!("{g:.6e}")
            }
            Err(e) => format!("ERR: {e}"),
        };
        println!(
            "{:<22} {:>16} {:>20} {:>8}",
            format!("{name} ({id})"),
            pos_s,
            gm_s,
            if pos.is_ok() && gm.is_ok() { "OK" } else { "PARTIAL" }
        );
    }

    println!("\nposition resolved: {pos_ok}/16   GM resolved: {gm_ok}/16");
    if gm_ok == 0 {
        println!("=> GMs are NOT in the shipped kernels; a hardcoded DE441 mass table is required.");
    }
}
