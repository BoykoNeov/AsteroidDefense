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
//! # These are the complete tables, and the reason that matters
//! Every knot of every vehicle's AMAT CSV is embedded here verbatim (101 / 10 /
//! 100 / 64 / 100 points). An earlier cut carried a ~10-point **downsample** of
//! each curve with a note claiming the resulting interpolation error was "well
//! under 1%". That claim had never been measured, and when it finally was, it was
//! wrong by roughly an order of magnitude: against the full tables the downsampled
//! Atlas V curve was off by **8.9%** near `C3 = 95`, Falcon Heavy reusable by 3.2%,
//! Vulcan by 2.7%. The curves are smooth in the middle but bend sharply as a
//! vehicle approaches its energy limit — exactly the high-`C3` region a fast
//! intercept lives in, so the error was concentrated where it mattered most.
//!
//! Two lessons are worth keeping next to the data. The estimate was plausible and
//! unmeasured, which is the same failure mode the sb441 GM provenance rules exist
//! to prevent; and the fix cost nothing, because the full CSVs were always one
//! fetch away. The shipped downsampled knots did each match the full table
//! *exactly* at their own `C3` values — the transcription was faithful, only the
//! sampling was too sparse.
//!
//! # The one caveat that remains, labelled as such
//! Delivered mass is modelled *as* impactor mass — no cruise-stage / bus /
//! propellant bookkeeping; that is a Phase-3 refinement (§8), and the mission layer
//! labels its outputs as patched-conic planning estimates accordingly.
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
// Knots transcribed **complete and verbatim** from AMAT `launcher-data/*.csv`
// (github.com/athulpg007/AMAT, machine-fetched via `gh api`, not retyped); source
// data from the NASA LSP Performance website via Girija arXiv:2310.05994. Payload
// in kg, C3 in km²/s². Every row of each CSV is present, in file order.

/// Atlas V 551 — a modest, real interplanetary launcher (flew New Horizons,
/// Juno). `launcher-data/atlas-v551.csv`.
pub const ATLAS_V_551: LaunchVehicle = LaunchVehicle {
    name: "Atlas V 551",
    knots: &[
        (0.0, 6114.504),
        (1.0, 5997.919),
        (2.0, 5881.334),
        (3.0, 5764.749),
        (4.0, 5652.021),
        (5.0, 5549.836),
        (6.0, 5447.652),
        (7.0, 5345.468),
        (8.0, 5243.284),
        (9.0, 5141.1),
        (10.0, 5051.076),
        (11.0, 4962.568),
        (12.0, 4874.06),
        (13.0, 4785.552),
        (14.0, 4697.044),
        (15.0, 4608.536),
        (16.0, 4520.028),
        (17.0, 4430.607),
        (18.0, 4340.682),
        (19.0, 4250.757),
        (20.0, 4160.832),
        (21.0, 4070.907),
        (22.0, 3980.982),
        (23.0, 3894.054),
        (24.0, 3814.92),
        (25.0, 3735.786),
        (26.0, 3656.652),
        (27.0, 3577.518),
        (28.0, 3498.384),
        (29.0, 3419.25),
        (30.0, 3347.66),
        (31.0, 3276.805),
        (32.0, 3205.951),
        (33.0, 3135.097),
        (34.0, 3064.243),
        (35.0, 2994.189),
        (36.0, 2931.913),
        (37.0, 2869.637),
        (38.0, 2807.36),
        (39.0, 2745.084),
        (40.0, 2682.807),
        (41.0, 2620.531),
        (42.0, 2558.255),
        (43.0, 2501.676),
        (44.0, 2447.112),
        (45.0, 2392.548),
        (46.0, 2337.985),
        (47.0, 2283.421),
        (48.0, 2228.857),
        (49.0, 2174.294),
        (50.0, 2125.119),
        (51.0, 2079.482),
        (52.0, 2033.845),
        (53.0, 1988.208),
        (54.0, 1942.572),
        (55.0, 1896.935),
        (56.0, 1851.298),
        (57.0, 1804.21),
        (58.0, 1756.066),
        (59.0, 1707.921),
        (60.0, 1659.777),
        (61.0, 1611.632),
        (62.0, 1563.488),
        (63.0, 1515.344),
        (64.0, 1471.3),
        (65.0, 1432.404),
        (66.0, 1393.508),
        (67.0, 1354.611),
        (68.0, 1315.715),
        (69.0, 1276.818),
        (70.0, 1237.922),
        (71.0, 1197.706),
        (72.0, 1157.445),
        (73.0, 1117.184),
        (74.0, 1076.922),
        (75.0, 1036.661),
        (76.0, 996.4),
        (77.0, 960.92),
        (78.0, 927.391),
        (79.0, 893.862),
        (80.0, 860.333),
        (81.0, 826.803),
        (82.0, 793.274),
        (83.0, 761.004),
        (84.0, 729.71),
        (85.0, 698.416),
        (86.0, 667.122),
        (87.0, 635.828),
        (88.0, 604.534),
        (89.0, 573.24),
        (90.0, 535.083),
        (91.0, 496.764),
        (92.0, 458.445),
        (93.0, 420.126),
        (94.0, 381.807),
        (95.0, 352.011),
        (96.0, 327.939),
        (97.0, 303.867),
        (98.0, 279.794),
        (99.0, 255.722),
        (100.0, 231.65),
    ],
};

/// Delta IV Heavy — the legacy high-energy heavy lifter (flew Parker Solar
/// Probe). `launcher-data/delta-IVH.csv` (sparse 10-point table, kept verbatim;
/// the sub-zero C3 knot is the fit's own and only [`LaunchVehicle::payload_kg`]
/// values at C3 ≥ 0 are ever queried).
pub const DELTA_IV_HEAVY: LaunchVehicle = LaunchVehicle {
    name: "Delta IV Heavy",
    knots: &[
        (-9.23755, 12032.841971383148),
        (0.178689, 10137.08108108108),
        (13.4342, 7901.729729729728),
        (25.5166, 6286.102384737678),
        (37.4095, 4957.468998410173),
        (49.193, 3854.6055788408703),
        (60.7237, 2956.515103338632),
        (72.9791, 2139.799227799227),
        (84.8127, 1460.2916193504407),
        (96.456, 875.5230524642266),
    ],
};

/// Falcon Heavy, expendable — the modern high-energy workhorse (flew Psyche;
/// baselined for flagship outer-planet studies). `launcher-data/falcon-heavy-expendable.csv`.
pub const FALCON_HEAVY_EXPENDABLE: LaunchVehicle = LaunchVehicle {
    name: "Falcon Heavy (expendable)",
    knots: &[
        (1.0, 14713.927),
        (2.0, 14443.836),
        (3.0, 14186.073),
        (4.0, 13933.79),
        (5.0, 13681.507),
        (6.0, 13429.224),
        (7.0, 13176.941),
        (8.0, 12924.658),
        (9.0, 12677.059),
        (10.0, 12433.475),
        (11.0, 12189.891),
        (12.0, 11948.827),
        (13.0, 11712.407),
        (14.0, 11475.988),
        (15.0, 11242.217),
        (16.0, 11021.862),
        (17.0, 10801.508),
        (18.0, 10581.154),
        (19.0, 10367.58),
        (20.0, 10159.817),
        (21.0, 9952.055),
        (22.0, 9744.292),
        (23.0, 9542.364),
        (24.0, 9357.686),
        (25.0, 9173.009),
        (26.0, 8988.331),
        (27.0, 8805.023),
        (28.0, 8624.962),
        (29.0, 8444.901),
        (30.0, 8264.84),
        (31.0, 8084.779),
        (32.0, 7909.233),
        (33.0, 7742.009),
        (34.0, 7574.786),
        (35.0, 7407.562),
        (36.0, 7240.339),
        (37.0, 7085.25),
        (38.0, 6932.368),
        (39.0, 6779.487),
        (40.0, 6626.605),
        (41.0, 6473.723),
        (42.0, 6321.157),
        (43.0, 6182.648),
        (44.0, 6044.14),
        (45.0, 5905.632),
        (46.0, 5767.123),
        (47.0, 5628.615),
        (48.0, 5490.107),
        (49.0, 5366.989),
        (50.0, 5248.78),
        (51.0, 5130.57),
        (52.0, 5012.36),
        (53.0, 4894.151),
        (54.0, 4775.941),
        (55.0, 4660.046),
        (56.0, 4547.854),
        (57.0, 4435.662),
        (58.0, 4323.47),
        (59.0, 4211.279),
        (60.0, 4099.087),
        (61.0, 3988.128),
        (62.0, 3877.321),
        (63.0, 3766.514),
        (64.0, 3655.708),
        (65.0, 3544.901),
        (66.0, 3451.158),
        (67.0, 3358.407),
        (68.0, 3265.656),
        (69.0, 3172.904),
        (70.0, 3080.153),
        (71.0, 2987.402),
        (72.0, 2901.065),
        (73.0, 2816.421),
        (74.0, 2731.777),
        (75.0, 2647.133),
        (76.0, 2562.489),
        (77.0, 2477.845),
        (78.0, 2395.917),
        (79.0, 2314.442),
        (80.0, 2232.966),
        (81.0, 2151.491),
        (82.0, 2070.015),
        (83.0, 1990.228),
        (84.0, 1915.434),
        (85.0, 1840.639),
        (86.0, 1765.845),
        (87.0, 1691.05),
        (88.0, 1616.256),
        (89.0, 1541.623),
        (90.0, 1467.041),
        (91.0, 1392.46),
        (92.0, 1317.878),
        (93.0, 1243.087),
        (94.0, 1168.062),
        (95.0, 1093.037),
        (96.0, 1018.011),
        (97.0, 944.849),
        (98.0, 873.43),
        (99.0, 802.012),
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
        (2.0, 6388.128),
        (3.0, 6218.84),
        (4.0, 6059.222),
        (5.0, 5901.826),
        (6.0, 5744.431),
        (7.0, 5588.715),
        (8.0, 5440.313),
        (9.0, 5291.911),
        (10.0, 5143.509),
        (11.0, 4997.888),
        (12.0, 4857.648),
        (13.0, 4717.409),
        (14.0, 4577.169),
        (15.0, 4436.929),
        (16.0, 4315.779),
        (17.0, 4195.738),
        (18.0, 4075.698),
        (19.0, 3955.657),
        (20.0, 3835.616),
        (21.0, 3719.067),
        (22.0, 3602.517),
        (23.0, 3485.967),
        (24.0, 3369.418),
        (25.0, 3260.274),
        (26.0, 3158.701),
        (27.0, 3057.128),
        (28.0, 2955.556),
        (29.0, 2853.983),
        (30.0, 2753.736),
        (31.0, 2654.577),
        (32.0, 2555.417),
        (33.0, 2456.258),
        (34.0, 2357.098),
        (35.0, 2263.615),
        (36.0, 2172.402),
        (37.0, 2081.189),
        (38.0, 1989.977),
        (39.0, 1899.65),
        (40.0, 1812.679),
        (41.0, 1725.709),
        (42.0, 1638.738),
        (43.0, 1551.768),
        (44.0, 1471.394),
        (45.0, 1398.066),
        (46.0, 1324.738),
        (47.0, 1251.41),
        (48.0, 1178.082),
        (49.0, 1104.754),
        (50.0, 1029.826),
        (51.0, 954.678),
        (52.0, 879.53),
        (53.0, 804.382),
        (54.0, 729.233),
        (55.0, 661.829),
        (56.0, 597.521),
        (57.0, 533.214),
        (58.0, 468.906),
        (59.0, 404.258),
        (60.0, 336.51),
        (61.0, 268.761),
        (62.0, 201.013),
        (63.0, 133.264),
        (64.0, 65.515),
    ],
};

/// Vulcan Centaur with 6 solids — the current-generation ULA launcher replacing
/// Atlas V and Delta IV. `launcher-data/vulcan-centaur-w-6-solids.csv`.
pub const VULCAN_CENTAUR: LaunchVehicle = LaunchVehicle {
    name: "Vulcan Centaur (6 solids)",
    knots: &[
        (1.0, 10529.589),
        (2.0, 10355.068),
        (3.0, 10181.544),
        (4.0, 10013.993),
        (5.0, 9846.443),
        (6.0, 9678.892),
        (7.0, 9521.211),
        (8.0, 9367.064),
        (9.0, 9212.918),
        (10.0, 9059.12),
        (11.0, 8906.032),
        (12.0, 8752.944),
        (13.0, 8599.856),
        (14.0, 8448.084),
        (15.0, 8303.554),
        (16.0, 8159.023),
        (17.0, 8014.493),
        (18.0, 7869.962),
        (19.0, 7725.201),
        (20.0, 7578.334),
        (21.0, 7431.467),
        (22.0, 7284.601),
        (23.0, 7137.734),
        (24.0, 6990.868),
        (25.0, 6844.001),
        (26.0, 6711.187),
        (27.0, 6583.697),
        (28.0, 6456.206),
        (29.0, 6328.715),
        (30.0, 6201.225),
        (31.0, 6087.352),
        (32.0, 5975.16),
        (33.0, 5862.968),
        (34.0, 5750.776),
        (35.0, 5638.584),
        (36.0, 5526.256),
        (37.0, 5413.718),
        (38.0, 5301.18),
        (39.0, 5188.642),
        (40.0, 5076.104),
        (41.0, 4965.095),
        (42.0, 4865.209),
        (43.0, 4765.323),
        (44.0, 4665.437),
        (45.0, 4565.551),
        (46.0, 4465.666),
        (47.0, 4365.915),
        (48.0, 4266.362),
        (49.0, 4166.809),
        (50.0, 4067.256),
        (51.0, 3967.704),
        (52.0, 3869.224),
        (53.0, 3777.808),
        (54.0, 3686.393),
        (55.0, 3594.977),
        (56.0, 3503.562),
        (57.0, 3412.146),
        (58.0, 3325.563),
        (59.0, 3241.729),
        (60.0, 3157.895),
        (61.0, 3074.061),
        (62.0, 2990.227),
        (63.0, 2906.393),
        (64.0, 2826.053),
        (65.0, 2751.572),
        (66.0, 2677.091),
        (67.0, 2602.61),
        (68.0, 2528.13),
        (69.0, 2453.649),
        (70.0, 2379.801),
        (71.0, 2306.473),
        (72.0, 2233.145),
        (73.0, 2159.817),
        (74.0, 2086.489),
        (75.0, 2013.161),
        (76.0, 1941.166),
        (77.0, 1869.248),
        (78.0, 1797.331),
        (79.0, 1725.413),
        (80.0, 1653.495),
        (81.0, 1582.441),
        (82.0, 1514.269),
        (83.0, 1446.097),
        (84.0, 1377.925),
        (85.0, 1309.753),
        (86.0, 1241.581),
        (87.0, 1173.409),
        (88.0, 1106.579),
        (89.0, 1045.195),
        (90.0, 983.811),
        (91.0, 922.426),
        (92.0, 861.042),
        (93.0, 799.087),
        (94.0, 736.128),
        (95.0, 673.17),
        (96.0, 610.212),
        (97.0, 553.116),
        (98.0, 498.12),
        (99.0, 443.124),
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
        // Halfway between two *adjacent* knots the linear interpolant is their mean
        // — pins the interpolation itself, not just the endpoints. Read off the
        // table rather than hardcoded, so it stays true whatever the knot spacing:
        // with the full CSVs the knots are 1 km²/s² apart, not 10.
        for v in LAUNCH_VEHICLES {
            let (x0, y0) = v.knots[0];
            let (x1, y1) = v.knots[1];
            let mid = v.payload_kg(0.5 * (x0 + x1));
            let expected = 0.5 * (y0 + y1);
            assert!(
                (mid - expected).abs() < 1e-6,
                "{}: midpoint {mid} vs {expected}",
                v.name
            );
        }
    }

    /// The full tables are *complete*, not a resample: each vehicle carries the row
    /// count of its source CSV. A silent re-downsample (or a truncated splice) would
    /// pass every other test here — interpolation, monotonicity, ordering all remain
    /// true of a subset — so the count is pinned explicitly.
    #[test]
    fn every_vehicle_carries_its_full_source_table() {
        let expected = [
            (ATLAS_V_551.name, 101),
            (DELTA_IV_HEAVY.name, 10),
            (FALCON_HEAVY_EXPENDABLE.name, 100),
            (FALCON_HEAVY_REUSABLE.name, 64),
            (VULCAN_CENTAUR.name, 100),
        ];
        for v in LAUNCH_VEHICLES {
            let want = expected
                .iter()
                .find(|(n, _)| *n == v.name)
                .unwrap_or_else(|| panic!("unlisted vehicle {}", v.name))
                .1;
            assert_eq!(v.knots.len(), want, "{} knot count", v.name);
        }
    }

    /// Knots are strictly ascending in `C3` — the invariant the bracketing scan in
    /// [`LaunchVehicle::payload_kg`] depends on, and the one a mis-ordered splice
    /// would break.
    #[test]
    fn knots_are_strictly_ascending_in_c3() {
        for v in LAUNCH_VEHICLES {
            for pair in v.knots.windows(2) {
                assert!(
                    pair[1].0 > pair[0].0,
                    "{}: knots out of order at C3={} → {}",
                    v.name,
                    pair[0].0,
                    pair[1].0
                );
            }
        }
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
