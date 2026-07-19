//! Probe the Tier-1 perturber field against a real DE kernel + constants set.
//!
//! For each of the ten MVP bodies (Sun + 8 planets + Moon, HANDOFF §5) this
//! prints whether ANISE resolves *both* an SSB position segment and a
//! gravitational parameter — the two facts the [`tier1_perturber_field`] builder
//! bakes in. It exists to confirm empirically (before 2c's ASSIST validation)
//! that the barycenter-vs-center frame choices in `TIER1_PERTURBER_FRAMES`
//! actually resolve against the shipped kernels, rather than trusting NAIF-id
//! recall. Provenance sibling of `probe_sun_gm` and `spike_geocenter`.
//!
//! Usage:
//!   cargo run -p asteroid_core --example probe_perturbers -- <de440s.bsp> <pck11.pca>
//! or set ASTEROID_DE_KERNEL and ASTEROID_PLANETARY_CONSTANTS and pass no args.

use std::sync::Arc;

use anise::constants::frames::{
    EARTH_J2000, JUPITER_BARYCENTER_J2000, MARS_BARYCENTER_J2000, MERCURY_J2000, MOON_J2000,
    NEPTUNE_BARYCENTER_J2000, SATURN_BARYCENTER_J2000, SSB_J2000, SUN_J2000,
    URANUS_BARYCENTER_J2000, VENUS_J2000,
};
use anise::prelude::{Epoch as AniseEpoch, Frame};
use anise::time::TimeScale;
use asteroid_core::ephemeris::{Ephemeris, KM3_S2_TO_M3_S2};
use asteroid_core::perturber_field::{tier1_perturber_field, KM_TO_M};

fn labeled_frames() -> [(&'static str, Frame); 10] {
    [
        ("Sun (10)", SUN_J2000),
        ("Mercury (199)", MERCURY_J2000),
        ("Venus (299)", VENUS_J2000),
        ("Earth geocenter (399)", EARTH_J2000),
        ("Moon (301)", MOON_J2000),
        ("Mars bary (4)", MARS_BARYCENTER_J2000),
        ("Jupiter bary (5)", JUPITER_BARYCENTER_J2000),
        ("Saturn bary (6)", SATURN_BARYCENTER_J2000),
        ("Uranus bary (7)", URANUS_BARYCENTER_J2000),
        ("Neptune bary (8)", NEPTUNE_BARYCENTER_J2000),
    ]
}

fn main() {
    let mut args = std::env::args().skip(1);
    let bsp = args
        .next()
        .or_else(|| asteroid_core::kernels::resolve().map(|k| k.bsp.display().to_string()))
        .unwrap_or_else(|| {
            eprintln!(
                "usage: probe_perturbers <de440s.bsp> <pck11.pca>  \
                 (or set ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS)"
            );
            std::process::exit(2);
        });
    let pca = args
        .next()
        .or_else(|| asteroid_core::kernels::resolve().map(|k| k.pca.display().to_string()))
        .unwrap_or_else(|| {
            eprintln!("missing planetary-constants (.pca) path");
            std::process::exit(2);
        });

    let eph = match Ephemeris::load(&bsp).and_then(|e| e.with_constants(&pca)) {
        Ok(e) => Arc::new(e),
        Err(e) => {
            eprintln!("FAILED to load kernels: {e}");
            std::process::exit(1);
        }
    };
    println!("Loaded DE kernel : {}", eph.kernel_path().display());
    println!("Loaded constants : {pca}\n");

    let epoch = AniseEpoch::from_gregorian(2020, 1, 1, 0, 0, 0, 0, TimeScale::TDB);
    println!("Epoch: {epoch}\n");
    println!(
        "{:<24} {:>16} {:>18} {:>10}",
        "body", "|r_SSB| (km)", "μ (km³/s²)", "status"
    );
    println!("{}", "-".repeat(72));

    let mut all_ok = true;
    for (label, frame) in labeled_frames() {
        let pos = eph.position_km(frame, SSB_J2000, epoch);
        let gm = eph.gm_km3_s2(frame);
        let (pos_s, gm_s, ok) = match (&pos, &gm) {
            (Ok(p), Ok(g)) => (format!("{:.1}", p.norm()), format!("{g:.6e}"), true),
            (Err(e), Ok(g)) => (format!("POS ERR: {e}"), format!("{g:.6e}"), false),
            (Ok(p), Err(e)) => (format!("{:.1}", p.norm()), format!("GM ERR: {e}"), false),
            (Err(pe), Err(ge)) => (format!("POS ERR: {pe}"), format!("GM ERR: {ge}"), false),
        };
        all_ok &= ok;
        println!(
            "{:<24} {:>16} {:>18} {:>10}",
            label,
            pos_s,
            gm_s,
            if ok { "OK" } else { "FAIL" }
        );
    }

    println!("\nBuilding tier1_perturber_field (SI, m³/s²) ...");
    match tier1_perturber_field(&eph) {
        Ok(field) => {
            println!(
                "  built {} perturbers; GM→SI factor {:.0e}, pos→SI factor {:.0e}",
                field.len(),
                KM3_S2_TO_M3_S2,
                KM_TO_M
            );
        }
        Err(e) => {
            all_ok = false;
            println!("  FAILED: {e}");
        }
    }

    println!(
        "\nPROBE (Tier-1 position + GM resolution): {}",
        if all_ok { "PASS" } else { "FAIL" }
    );
    std::process::exit(if all_ok { 0 } else { 1 });
}
