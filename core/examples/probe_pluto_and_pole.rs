//! Probe the two data questions this batch's leftovers turn on.
//!
//! **1. Pluto.** HANDOFF's open-questions list parks "Pluto in the shipping
//! perturber field" on one blocker: the shipped `pck11.pca` carries **no
//! `BODY9_GM`**, so an 11th perturber needed a DE441-consistent GM from somewhere.
//! It has one — `GM9` in the DE440 header's own constant record — and this probe
//! answers the two empirical halves: does `de440s.bsp` carry a Pluto-barycenter
//! *position*, and does ANISE resolve a *GM* for that frame (i.e. is the hardcode
//! actually necessary, and if pck11 does carry one, do the two agree)?
//!
//! **2. Earth's pole.** The `J2` term is defined about Earth's **spin axis**, not
//! the frame's `z`. This reports the ICRF pole ANISE rotates out of the loaded
//! planetary constants at several epochs, so the "close to `ẑ` but not `ẑ`"
//! claim in `forces::oblateness` is a measured number rather than an assumption.
//!
//! Usage: set ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS and run:
//!   cargo run -p asteroid_core --example probe_pluto_and_pole

use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME, PLUTO_BARYCENTER_J2000, SSB_J2000};
use anise::prelude::Epoch as AniseEpoch;
use anise::time::TimeScale;
use asteroid_core::ephemeris::Ephemeris;

/// `GM9` (Pluto system barycenter), au³/day², read verbatim out of the local
/// `linux_p1550p2650.440` constant record.
const GM9_AU3_DAY2: f64 = 2.175_096_464_893_358e-12;

const AU_KM: f64 = 1.495_978_707e8;
const DAY_S: f64 = 86_400.0;

fn main() {
    let Some(k) = asteroid_core::kernels::resolve() else {
        eprintln!("no DE kernel pair resolved");
        std::process::exit(2);
    };
    let eph = Ephemeris::load(&k.bsp)
        .and_then(|e| e.with_constants(&k.pca))
        .unwrap_or_else(|e| {
            eprintln!("FAILED to load kernels: {e}");
            std::process::exit(1);
        });
    println!("DE kernel : {}", k.bsp.display());
    println!("constants : {}\n", k.pca.display());

    let epoch = AniseEpoch::from_gregorian(2030, 1, 1, 0, 0, 0, 0, TimeScale::TDB);

    // --- 1. Pluto ------------------------------------------------------------
    println!("--- Pluto barycenter (NAIF 9) ---");
    match eph.position_km(PLUTO_BARYCENTER_J2000, SSB_J2000, epoch) {
        Ok(r) => println!("position  : |r| = {:.4} AU  (SSB, 2030-01-01 TDB)", r.norm() / AU_KM),
        Err(e) => println!("position  : NOT AVAILABLE — {e}"),
    }
    let hardcoded_km3_s2 = GM9_AU3_DAY2 * AU_KM.powi(3) / (DAY_S * DAY_S);
    println!("DE440 GM9 : {GM9_AU3_DAY2:.15e} au³/day² = {hardcoded_km3_s2:.6} km³/s²");
    match eph.gm_km3_s2(PLUTO_BARYCENTER_J2000) {
        Ok(mu) => {
            let rel = (mu - hardcoded_km3_s2).abs() / hardcoded_km3_s2;
            println!("pck11  GM : {mu:.6} km³/s²  (relative difference {:.3}%)", rel * 100.0);
        }
        Err(e) => println!("pck11  GM : NOT RESOLVED — {e}\n            (this is why GM9 is hardcoded)"),
    }

    // --- 2. Earth's spin axis in ICRF ---------------------------------------
    println!("\n--- Earth pole in ICRF (from the loaded orientation data) ---");
    for year in [2000, 2020, 2040, 2100] {
        let e = AniseEpoch::from_gregorian(year, 1, 1, 0, 0, 0, 0, TimeScale::TDB);
        match eph.pole_unit_icrf(IAU_EARTH_FRAME, EARTH_J2000, e) {
            Ok(pole) => {
                let (x, y, z) = (pole[0], pole[1], pole[2]);
                let tilt_deg = z.clamp(-1.0, 1.0).acos().to_degrees();
                let ra = y.atan2(x).to_degrees();
                let dec = z.clamp(-1.0, 1.0).asin().to_degrees();
                println!(
                    "{year}: pole = ({x:+.8}, {y:+.8}, {z:+.8})  RA {ra:+.4}°  Dec {dec:+.4}°  \
                     tilt from ẑ {tilt_deg:.4}°"
                );
            }
            Err(err) => println!("{year}: rotation NOT AVAILABLE — {err}"),
        }
    }
}
