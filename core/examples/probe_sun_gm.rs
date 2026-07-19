//! Print the Sun's gravitational parameter as ANISE resolves it from a loaded
//! planetary-constants set (`.pca`).
//!
//! This is the provenance step for the §10.6 reference fixture: the exact μ ANISE
//! returns here is baked into `pyref/generate_kepler_fixture.py`, so hapsira and
//! the Rust `KeplerPropagator` propagate the *same* orbit (HANDOFF §6, "pull GM
//! through ANISE"). Run once when the constants set changes; the gated test
//! `sun_gm_matches_fixture` then guards the pinned value on every CI run that
//! has the `.pca` available.
//!
//! Usage:
//!   cargo run -p asteroid_core --example probe_sun_gm -- <path/to/pck11.pca>
//! or set ASTEROID_PLANETARY_CONSTANTS and pass no argument.

use asteroid_core::ephemeris::Ephemeris;

fn main() {
    let pca = std::env::args()
        .nth(1)
        .or_else(|| asteroid_core::kernels::resolve().map(|k| k.pca.display().to_string()))
        .unwrap_or_else(|| {
            eprintln!("usage: probe_sun_gm <pck11.pca>  (or set ASTEROID_PLANETARY_CONSTANTS)");
            std::process::exit(2);
        });

    let eph = match Ephemeris::load(&pca) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("FAILED to load constants: {e}");
            std::process::exit(1);
        }
    };

    match (
        eph.gm_km3_s2(anise::constants::frames::SUN_J2000),
        eph.sun_gm_m3_s2(),
    ) {
        (Ok(km), Ok(si)) => {
            println!("Sun GM (ANISE, from {pca}):");
            println!("  {km:.6} km^3/s^2");
            println!("  {si:.17e} m^3/s^2   <- bake this into the fixture generator");
        }
        (Err(e), _) | (_, Err(e)) => {
            eprintln!("FAILED to resolve Sun GM: {e}");
            std::process::exit(1);
        }
    }
}
