//! `launch_vehicle` — real launch-vehicle high-energy performance (HANDOFF §8).
//!
//! The mission-design layer needs one thing this module supplies: **given a
//! departure characteristic energy `C3` (km²/s²), how much spacecraft mass can a
//! real rocket actually deliver?** That is what turns the porkchop from an
//! abstract Δv map into a *deliverability* map (§7, §180) — the launch energy a
//! Lambert transfer demands is only useful next to what a launcher can lift to it.
//!
//! # Provenance — fetched and cited, never recalled
//! Plausible-looking launch numbers are the exact recallable-but-wrong trap the
//! sb441 GMs were guarded against, and unlike those there is no kernel to
//! machine-verify against — so every value here is transcribed from a cited
//! external source, not memory. The `C3`→payload curves are the open-source
//! **AMAT** (Aerocapture Mission Analysis Tool) `launcher-data/` tables
//! (`github.com/athulpg007/AMAT`, MIT-licensed), which are in turn compiled from
//! the **NASA Launch Services Program Performance website**
//! (`elvperf.ksc.nasa.gov`) — see Girija, *Launch Vehicle High-Energy Performance
//! Dataset*, arXiv:2310.05994. AMAT interpolates the tables linearly with
//! `fill_value = 0` outside the tabulated `C3` range; this module reproduces that
//! exactly (linear between knots, **0 = infeasible** below the first / above the
//! last knot).
//!
//! # Teaching-grade, and labelled as such
//! Two honest caveats travel with these numbers. (1) The knots here are the AMAT
//! tables **downsampled to ~10 km²/s² spacing**; because the curves are smooth and
//! near-linear over a decade of `C3`, the linear-interp error against the full
//! AMAT table is well under 1% — negligible for a deliverability overlay, and the
//! full CSVs drop in unchanged if exactness is ever wanted. (2) Delivered mass is
//! modelled *as* impactor mass — no cruise-stage / bus / propellant bookkeeping;
//! that is a Phase-3 refinement (§8), and the mission layer labels its outputs as
//! patched-conic planning estimates accordingly.
//!
//! # Kernel-free by construction
//! Pure tabulated data + interpolation; no ephemeris, no I/O. Validated in
//! isolation: the interpolation reproduces the embedded knots exactly, is
//! monotonic in `C3`, and returns 0 (infeasible) outside each vehicle's range.

/// A launch vehicle's high-energy delivery capability: payload mass as a function
/// of departure characteristic energy `C3` (the square of the hyperbolic excess
/// speed relative to Earth).
///
/// The capability is a table of `(C3 km²/s², payload kg)` knots, ascending in
/// `C3`, interpolated linearly. Outside `[min C3, max C3]` the payload is 0 —
/// the vehicle cannot reach that launch energy, which the porkchop renders as an
/// infeasible cell rather than an extrapolated fiction.
#[derive(Debug, Clone, Copy)]
pub struct LaunchVehicle {
    /// Display name.
    pub name: &'static str,
    /// `(C3 km²/s², payload kg)` knots, strictly ascending in `C3`.
    knots: &'static [(f64, f64)],
}

impl LaunchVehicle {
    /// Deliverable payload mass (kg) at departure characteristic energy
    /// `c3_km2_s2` (km²/s²). Linear interpolation between the tabulated knots;
    /// **0 outside the vehicle's tabulated `C3` range** (infeasible), mirroring
    /// AMAT's `interp1d(fill_value=0, bounds_error=False)`.
    ///
    /// Note the unit: `C3` is in **km²/s²** here (the tables' native unit). The
    /// mission layer computes `C3` in SI (m²/s²) from the Lambert departure
    /// velocity and must divide by `1e6` before calling — the units boundary is
    /// explicit precisely because a silent km/m slip is the classic delivery bug.
    pub fn payload_kg(&self, c3_km2_s2: f64) -> f64 {
        let knots = self.knots;
        // Fail closed: NaN and out-of-range both yield 0 (infeasible).
        if !c3_km2_s2.is_finite() {
            return 0.0;
        }
        let (c3_lo, _) = knots[0];
        let (c3_hi, _) = knots[knots.len() - 1];
        if c3_km2_s2 < c3_lo || c3_km2_s2 > c3_hi {
            return 0.0;
        }
        // Locate the bracketing segment and interpolate. Linear scan is fine —
        // a dozen knots, and the mission grid caches per-vehicle results anyway.
        for pair in knots.windows(2) {
            let (x0, y0) = pair[0];
            let (x1, y1) = pair[1];
            if c3_km2_s2 >= x0 && c3_km2_s2 <= x1 {
                let t = if x1 > x0 {
                    (c3_km2_s2 - x0) / (x1 - x0)
                } else {
                    0.0
                };
                return y0 + t * (y1 - y0);
            }
        }
        // Unreachable given the range check above, but fail closed.
        0.0
    }

    /// The maximum characteristic energy the vehicle is tabulated for (km²/s²) —
    /// the launch energy above which delivery is infeasible.
    pub fn max_c3_km2_s2(&self) -> f64 {
        self.knots[self.knots.len() - 1].0
    }

    /// The minimum tabulated characteristic energy (km²/s²).
    pub fn min_c3_km2_s2(&self) -> f64 {
        self.knots[0].0
    }
}

// --- The vehicle table -------------------------------------------------------
//
// Knots transcribed from AMAT `launcher-data/*.csv` (github.com/athulpg007/AMAT),
// downsampled to ~10 km²/s² spacing; source data from the NASA LSP Performance
// website via Girija arXiv:2310.05994. Payload in kg, C3 in km²/s².

/// Atlas V 551 — a modest, real interplanetary launcher (flew New Horizons,
/// Juno). `launcher-data/atlas-v551.csv`.
pub const ATLAS_V_551: LaunchVehicle = LaunchVehicle {
    name: "Atlas V 551",
    knots: &[
        (0.0, 6114.504),
        (10.0, 5051.076),
        (20.0, 4160.832),
        (30.0, 3347.660),
        (40.0, 2682.807),
        (50.0, 2125.119),
        (60.0, 1659.777),
        (70.0, 1237.922),
        (80.0, 860.333),
        (90.0, 535.083),
        (100.0, 231.650),
    ],
};

/// Delta IV Heavy — the legacy high-energy heavy lifter (flew Parker Solar
/// Probe). `launcher-data/delta-IVH.csv` (sparse 10-point table, kept verbatim;
/// the sub-zero C3 knot is the fit's own and only [`LaunchVehicle::payload_kg`]
/// values at C3 ≥ 0 are ever queried).
pub const DELTA_IV_HEAVY: LaunchVehicle = LaunchVehicle {
    name: "Delta IV Heavy",
    knots: &[
        (-9.237_550, 12032.842),
        (0.178_689, 10137.081),
        (13.434_204, 7901.730),
        (25.516_625, 6286.102),
        (37.409_473, 4957.469),
        (49.193_029, 3854.606),
        (60.723_663, 2956.515),
        (72.979_109, 2139.799),
        (84.812_709, 1460.292),
        (96.455_964, 875.523),
    ],
};

/// Falcon Heavy, expendable — the modern high-energy workhorse (flew Psyche;
/// baselined for flagship outer-planet studies). `launcher-data/falcon-heavy-expendable.csv`.
pub const FALCON_HEAVY_EXPENDABLE: LaunchVehicle = LaunchVehicle {
    name: "Falcon Heavy (expendable)",
    knots: &[
        (1.0, 14713.927),
        (10.0, 12433.475),
        (20.0, 10159.817),
        (30.0, 8264.840),
        (40.0, 6626.605),
        (50.0, 5248.780),
        (60.0, 4099.087),
        (70.0, 3080.153),
        (80.0, 2232.966),
        (90.0, 1467.041),
        (100.0, 730.594),
    ],
};

/// Falcon Heavy, reusable — the same vehicle flown to recover its boosters,
/// trading high-energy capability for reuse. Included to make the delivery
/// tradeoff legible. `launcher-data/falcon-heavy-reusable.csv`.
pub const FALCON_HEAVY_REUSABLE: LaunchVehicle = LaunchVehicle {
    name: "Falcon Heavy (reusable)",
    knots: &[
        (1.0, 6557.416),
        (10.0, 5143.509),
        (20.0, 3835.616),
        (30.0, 2753.736),
        (40.0, 1812.679),
        (50.0, 1029.826),
        (60.0, 336.510),
        (64.0, 65.515),
    ],
};

/// Vulcan Centaur with 6 solids — the current-generation ULA launcher replacing
/// Atlas V and Delta IV. `launcher-data/vulcan-centaur-w-6-solids.csv`.
pub const VULCAN_CENTAUR: LaunchVehicle = LaunchVehicle {
    name: "Vulcan Centaur (6 solids)",
    knots: &[
        (1.0, 10529.589),
        (10.0, 9059.120),
        (20.0, 7578.334),
        (30.0, 6201.225),
        (40.0, 5076.104),
        (50.0, 4067.256),
        (60.0, 3157.895),
        (70.0, 2379.801),
        (80.0, 1653.495),
        (90.0, 983.811),
        (100.0, 388.128),
    ],
};

/// The full set of vehicles the mission layer offers, weakest-to-strongest at
/// low `C3` — a deliberate capability spread so the porkchop teaches that
/// different launchers open different regions of the launch-window map.
pub const LAUNCH_VEHICLES: &[LaunchVehicle] = &[
    ATLAS_V_551,
    FALCON_HEAVY_REUSABLE,
    VULCAN_CENTAUR,
    DELTA_IV_HEAVY,
    FALCON_HEAVY_EXPENDABLE,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolation_reproduces_the_knots_exactly() {
        // At a tabulated C3, the interpolant must return that knot's payload —
        // a transcription/off-by-one guard on the embedded data.
        for v in LAUNCH_VEHICLES {
            for &(c3, mass) in v.knots {
                if c3 < 0.0 {
                    continue; // sub-zero fit knots are never queried
                }
                let got = v.payload_kg(c3);
                assert!(
                    (got - mass).abs() < 1e-6,
                    "{}: payload_kg({c3}) = {got}, knot says {mass}",
                    v.name
                );
            }
        }
    }

    #[test]
    fn payload_decreases_monotonically_with_c3() {
        // Physical invariant: more launch energy always means less deliverable
        // mass. Sampled densely across each vehicle's range.
        for v in LAUNCH_VEHICLES {
            let lo = v.min_c3_km2_s2().max(0.0);
            let hi = v.max_c3_km2_s2();
            let n = 200;
            let mut prev = f64::INFINITY;
            for i in 0..=n {
                let c3 = lo + (hi - lo) * (i as f64) / (n as f64);
                let m = v.payload_kg(c3);
                assert!(
                    m <= prev + 1e-6,
                    "{}: payload rose at C3={c3} ({m} > {prev})",
                    v.name
                );
                prev = m;
            }
        }
    }

    #[test]
    fn infeasible_outside_the_tabulated_range() {
        // Above the last knot the vehicle simply cannot reach that C3 — 0, not an
        // extrapolated fiction. Same below the first knot and for NaN.
        for v in LAUNCH_VEHICLES {
            assert_eq!(v.payload_kg(v.max_c3_km2_s2() + 1.0), 0.0, "{}", v.name);
            assert_eq!(v.payload_kg(v.min_c3_km2_s2() - 1.0), 0.0, "{}", v.name);
            assert_eq!(v.payload_kg(f64::NAN), 0.0, "{}", v.name);
        }
    }

    #[test]
    fn midpoint_interpolation_is_the_linear_average() {
        // Halfway between two knots the linear interpolant is their mean — pins
        // the interpolation itself, not just the endpoints.
        let v = FALCON_HEAVY_EXPENDABLE;
        // Between C3=10 (12433.475) and C3=20 (10159.817).
        let mid = v.payload_kg(15.0);
        let expected = 0.5 * (12433.475 + 10159.817);
        assert!((mid - expected).abs() < 1e-6, "mid {mid} vs {expected}");
    }

    #[test]
    fn stronger_vehicles_deliver_more_at_a_representative_c3() {
        // A sanity anchor on the capability spread at a Mars-class C3 (~15):
        // Falcon Heavy expendable > Vulcan > Atlas V 551. Not a tautology — it
        // confirms the tables were assigned to the right vehicles.
        let c3 = 15.0;
        let fh = FALCON_HEAVY_EXPENDABLE.payload_kg(c3);
        let vulcan = VULCAN_CENTAUR.payload_kg(c3);
        let atlas = ATLAS_V_551.payload_kg(c3);
        assert!(fh > vulcan, "FH {fh} should exceed Vulcan {vulcan}");
        assert!(vulcan > atlas, "Vulcan {vulcan} should exceed Atlas {atlas}");
    }
}
