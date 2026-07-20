//! ANISE-backed perturber field — the bridge from the [`Ephemeris`] loader to the
//! composable [`ForceModel`](crate::forces::ForceModel) (HANDOFF §5, §10.7).
//!
//! [`point_mass`](crate::forces::point_mass) defines *what* a point-mass field is
//! (the [`PerturberEphemeris`] contract + the term that sums `μ·r̂/r²`); it stays
//! deliberately **ANISE-free** so its unit tests need no kernel. This module
//! supplies the missing half: an [`EphemerisPerturber`] that answers
//! [`PerturberEphemeris::position_at`] from a loaded DE kernel, and a
//! [`tier1_perturber_field`] builder that assembles the MVP Sun + 8 planets +
//! Moon field from it.
//!
//! # Frame and units
//! [`Ephemeris`] queries return **kilometres** in the SSB-centered ICRF (J2000)
//! frame; the core physics runs in **SI metres** in that same barycentric frame
//! (HANDOFF §5). Positions cross the boundary through [`KM_TO_M`] (`1e3`), GM
//! through [`KM3_S2_TO_M3_S2`] (`1e9`) — kept as separate constants so a position
//! can never be scaled by the GM factor.
//!
//! # The two §5 footguns, both handled by frame choice
//! - **Barycentric, not heliocentric.** Every perturber position is SSB-relative
//!   (`observer = SSB_J2000`), so the integration frame is inertial and owes no
//!   indirect term.
//! - **Geocenter, not EMB.** [`TIER1_PERTURBER_FRAMES`] carries Earth as the
//!   reconstructed geocenter ([`EARTH_J2000`], NAIF 399) and the Moon
//!   ([`MOON_J2000`], 301) as a *separate* perturber — never the Earth–Moon
//!   barycenter (NAIF 3), whose ~4671 km offset would corrupt the b-plane.
//!
//! # Position ↔ GM pairing
//! [`tier1_perturber_field`] drives both the position source and the μ lookup
//! from the *same* [`Frame`] per body, so the mass a perturber's μ describes is
//! structurally guaranteed to match the body its position tracks (a barycenter
//! position is paired with that barycenter's GM; the 399 geocenter with
//! Earth-only GM). Getting that pairing wrong is the classic silent
//! wrong-force-field bug this construction makes unrepresentable.

use std::sync::Arc;

use anise::constants::frames::{
    EARTH_J2000, JUPITER_BARYCENTER_J2000, MARS_BARYCENTER_J2000, MERCURY_J2000, MOON_J2000,
    NEPTUNE_BARYCENTER_J2000, SATURN_BARYCENTER_J2000, SSB_J2000, SUN_J2000,
    URANUS_BARYCENTER_J2000, VENUS_J2000,
};
use anise::prelude::Frame;
use nalgebra::Vector3;

use anise::prelude::Epoch as AniseEpoch;
use anise::time::TimeScale;

use crate::ephemeris::{Ephemeris, EphemerisError, KM3_S2_TO_M3_S2};
use crate::epoch::Epoch;
use crate::forces::point_mass::{Perturber, PerturberEphemeris, PointMassGravity};
use crate::forces::ForceError;
use crate::state::StateVector;

/// SI conversion for **positions**: 1 km in metres. Separate from
/// [`KM3_S2_TO_M3_S2`] (the GM factor, `1e9`) so the two can never be confused.
pub const KM_TO_M: f64 = 1.0e3;

/// One astronomical unit in **kilometres** — the DE440 header value
/// (`AU = 0.149597870699999988D+09 km`), used only to convert the sb441 asteroid
/// GMs out of the JPL-native au³/day² unit. The same figure `1.495_978_707e11` m
/// the scenario/validation fixtures use, expressed in km.
const AU_KM: f64 = 1.495_978_707e8;

/// Seconds in a day, for the au³/**day²** → km³/**s²** conversion of the asteroid GMs.
const DAY_S: f64 = 86_400.0;

/// Convert a gravitational parameter from JPL's **au³/day²** (the unit the DE440
/// header carries the asteroid `MA%04d` masses in) to **km³/s²** (the unit ANISE
/// carries planetary GMs in, and the one the km→SI factor [`KM3_S2_TO_M3_S2`]
/// consumes). Kept as a named constant so the single multiply is auditable, and
/// so the unit conversion has one home a test can pin against pck11's independently
/// determined km³/s² values (see [`sb441_perturber_field`]'s tests).
const AU3_DAY2_TO_KM3_S2: f64 = (AU_KM * AU_KM * AU_KM) / (DAY_S * DAY_S);

/// The 16 DE441 main-belt perturbers — the asteroid force field ASSIST integrates
/// against (HANDOFF §5, Tier-2), each as `(NAIF id, name, GM in au³/day²)`.
///
/// **The masses are the load-bearing half, and their provenance is deliberate.**
/// `sb441-n16.bsp` supplies only *positions*; ASSIST joins the masses from the
/// DE440/441 planetary file's own constants (`MA%04d`, keyed by asteroid number),
/// so the mass paired with each position is the very one JPL used when it
/// *integrated* that position — using any other value would fly a perturber whose
/// gravity disagrees with the trajectory it traces. These sixteen are transcribed
/// **verbatim** from the DE440 header's GROUP 1041 (`MA0001 … MA0704`, D→e), and
/// were re-read straight out of the local `linux_p1550p2650.440` binary's constant
/// record to confirm the on-disk kernel carries these exact doubles — not recalled,
/// not from documentation. The three best-determined (Ceres, Pallas, Vesta) match
/// the independent pck11 SPICE constants to <1% (the unit-conversion guard, since a
/// wrong au³/day² factor would miss by orders of magnitude); the rest are DE440's
/// own free-fit solution and legitimately differ from later spacecraft/occultation
/// determinations by tens of percent — which is the whole reason to use *this* set.
///
/// NAIF ids follow the numbered-asteroid convention `2000000 + number`, so 2000001
/// is (1) Ceres. Ordered by number, matching [`crate::perturber_field`]'s field.
///
/// The literals carry the DE440 header's full precision — more digits than an `f64`
/// resolves, the last rounding away — kept **verbatim** for provenance exactly as
/// [`DE440_EMRAT`](crate::ephemeris::DE440_EMRAT) is; clippy confirms the truncation
/// is bit-identical, so the stored values are unchanged either way.
#[allow(clippy::excessive_precision)]
pub const SB441_PERTURBER_GM_AU3_DAY2: [(i32, &str, f64); 16] = [
    (2000001, "Ceres", 0.139645181230810698e-12),
    (2000002, "Pallas", 0.304711463300432000e-13),
    (2000003, "Juno", 0.428234396779950106e-14),
    (2000004, "Vesta", 0.385480002252579039e-13),
    (2000007, "Iris", 0.254160149734714977e-14),
    (2000010, "Hygiea", 0.125425307616408099e-13),
    (2000015, "Eunomia", 0.451077990514367950e-14),
    (2000016, "Psyche", 0.354450028424889778e-14),
    (2000031, "Euphrosyne", 0.240670122189375765e-14),
    (2000052, "Europa", 0.598243152648698406e-14),
    (2000065, "Cybele", 0.209171759551336823e-14),
    (2000087, "Sylvia", 0.483456065461055208e-14),
    (2000088, "Thisbe", 0.265294366103563534e-14),
    (2000107, "Camilla", 0.321913920758785882e-14),
    (2000511, "Davida", 0.868362534922865448e-14),
    (2000704, "Interamnia", 0.631103434208788874e-14),
];

/// The Tier-1 MVP perturber set (HANDOFF §5): Sun + 8 planets + Moon, ten bodies.
///
/// Each entry is the [`Frame`] used for **both** that body's position and its GM
/// (see the module-level "Position ↔ GM pairing" note). Frame choices encode the
/// §5 rules:
/// - **Earth is the geocenter** ([`EARTH_J2000`], 399) and the **Moon is separate**
///   ([`MOON_J2000`], 301) — the EMB (NAIF 3) never appears.
/// - **Mercury / Venus** use their body centers (199 / 299): they have no
///   significant satellites, so center and barycenter coincide.
/// - **Mars … Neptune** use their system **barycenters** (NAIF 4–8): a DE kernel
///   carries the giants only as barycenters, and lumping each planet's moons into
///   the barycenter mass is the standard N-body treatment — **confirmed ASSIST's**
///   in the batch-2c validation (`validation/tests/assist_reference.rs`), where a
///   test particle in this field reproduces ASSIST's track to ~4.5e-11 relative
///   over two years.
///
/// **One deliberate difference from ASSIST, quantified in 2c:** ASSIST's
/// point-mass term also carries **Pluto** (its 11th body). This shipping field
/// omits it, per §5's locked "Sun + 8 planets + Moon". The measured cost is
/// ~55 m over two years for a main-belt test particle (growing with lead time);
/// Pluto joins at Tier 2 alongside the 16 asteroid perturbers (which also need a
/// DE441-consistent GM source — `pck11.pca` carries no Pluto GM). The 2c test adds
/// Pluto to its *comparison* field so both sides integrate ASSIST's identical
/// 11-body system, isolating the machinery's correctness from that omission.
pub const TIER1_PERTURBER_FRAMES: [Frame; 10] = [
    SUN_J2000,
    MERCURY_J2000,
    VENUS_J2000,
    EARTH_J2000, // 399 geocenter, reconstructed — NOT the EMB (§5 footgun)
    MOON_J2000,  // 301, carried separately through the encounter
    MARS_BARYCENTER_J2000,
    JUPITER_BARYCENTER_J2000,
    SATURN_BARYCENTER_J2000,
    URANUS_BARYCENTER_J2000,
    NEPTUNE_BARYCENTER_J2000,
];

/// A single ANISE-backed perturber position source: one body (identified by
/// `frame`) whose SSB-relative position is looked up from a shared [`Ephemeris`]
/// on demand.
///
/// Implements [`PerturberEphemeris`] so it drops straight into
/// [`PointMassGravity`]. The [`Ephemeris`] is shared via [`Arc`] because the
/// whole Tier-1 field (ten perturbers) reads from one loaded almanac; `Arc`
/// (over `Rc`) costs nothing here and leaves the door open to parallel lead-time
/// sweeps later without re-plumbing ownership.
#[derive(Clone)]
pub struct EphemerisPerturber {
    ephemeris: Arc<Ephemeris>,
    frame: Frame,
}

impl EphemerisPerturber {
    /// A perturber that tracks `frame`'s position (relative to the SSB) via the
    /// shared `ephemeris`.
    pub fn new(ephemeris: Arc<Ephemeris>, frame: Frame) -> Self {
        Self { ephemeris, frame }
    }

    /// The body frame this perturber tracks.
    pub fn frame(&self) -> Frame {
        self.frame
    }
}

impl EphemerisPerturber {
    /// SSB-relative **state** (position m, velocity m/s) of `self.frame` at
    /// `epoch`, barycentric ICRF, SI. The full-state companion to
    /// [`PerturberEphemeris::position_at`]: the close-approach detector needs a
    /// perturber's *velocity* too (Earth's, to form `v_rel`), which the point-mass
    /// force term never asks for. Maps an [`EphemerisError`] to
    /// [`ForceError::Ephemeris`] so a failed lookup fails loudly.
    pub fn state_at(&self, epoch: Epoch) -> Result<StateVector, ForceError> {
        let (r_km, v_km_s) = self
            .ephemeris
            .state_km_s(self.frame, SSB_J2000, epoch.as_hifitime())
            .map_err(|e| ForceError::Ephemeris(e.to_string()))?;
        // anise::math::Vector3 IS nalgebra::Vector3<f64>; km→m and km/s→m/s scales.
        Ok(StateVector::new(r_km * KM_TO_M, v_km_s * KM_TO_M))
    }
}

impl PerturberEphemeris for EphemerisPerturber {
    /// SSB-relative position of `self.frame` at `epoch`, in **metres**,
    /// barycentric ICRF. Maps an [`EphemerisError`] to a
    /// [`ForceError::Ephemeris`] so a failed ephemeris lookup fails the force
    /// evaluation loudly rather than injecting a bogus position.
    fn position_at(&self, epoch: Epoch) -> Result<Vector3<f64>, ForceError> {
        let position_km = self
            .ephemeris
            .position_km(self.frame, SSB_J2000, epoch.as_hifitime())
            .map_err(|e| ForceError::Ephemeris(e.to_string()))?;
        // anise::math::Vector3 IS nalgebra::Vector3<f64>, so this is a plain
        // km→m scale, no type conversion.
        Ok(position_km * KM_TO_M)
    }
}

impl crate::close_approach::GeocentricState for EphemerisPerturber {
    /// The SSB-relative state of this perturber's body — Earth's geocentre when
    /// built with [`EARTH_J2000`]. Lets an [`EphemerisPerturber`] serve directly
    /// as the close-approach detector's Earth-state source.
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, ForceError> {
        EphemerisPerturber::state_at(self, epoch)
    }
}

impl crate::forces::relativity::CentralBodyState for EphemerisPerturber {
    /// The SSB-relative **state** of this perturber's body — the Sun's when built
    /// with [`SUN_J2000`]. This is what the heliocentric-referenced Tier-2 terms
    /// (1PN relativity, Yarkovsky) subtract to form the body's `r`,`v` about the
    /// Sun. The isolation tests use [`FixedCentralBody`](crate::FixedCentralBody)
    /// at the origin; the *shipping* field wants the Sun's real barycentric
    /// wobble, which is exactly this lookup — the same one
    /// [`state_at`](EphemerisPerturber::state_at) already serves the close-approach
    /// detector, so GR/Yarkovsky and the encounter geometry read one Sun.
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, ForceError> {
        EphemerisPerturber::state_at(self, epoch)
    }
}

/// Assemble the Tier-1 MVP point-mass field (Sun + 8 planets + Moon) from a
/// loaded [`Ephemeris`] (HANDOFF §5, §10.7).
///
/// For each frame in [`TIER1_PERTURBER_FRAMES`] this pairs an
/// [`EphemerisPerturber`] (position source) with the μ ANISE resolves for the
/// *same* frame — so position and mass describe the same body by construction.
/// GM crosses to SI via [`KM3_S2_TO_M3_S2`].
///
/// Fails loud with [`EphemerisError::Constants`] if any body's GM does not
/// resolve (e.g. only a `.bsp` was loaded but not a planetary-constants `.pca`),
/// rather than silently dropping a perturber — an incomplete field is a wrong
/// field.
pub fn tier1_perturber_field(
    ephemeris: &Arc<Ephemeris>,
) -> Result<PointMassGravity, EphemerisError> {
    let mut perturbers = Vec::with_capacity(TIER1_PERTURBER_FRAMES.len());
    for &frame in &TIER1_PERTURBER_FRAMES {
        let mu = ephemeris.gm_km3_s2(frame)? * KM3_S2_TO_M3_S2;
        let source = EphemerisPerturber::new(Arc::clone(ephemeris), frame);
        perturbers.push(Perturber::new(mu, source));
    }
    Ok(PointMassGravity::new(perturbers))
}

/// Assemble the 16 sb441 asteroid perturbers as a point-mass field (HANDOFF §5,
/// Tier-2) — the main-belt bodies that set the residual floor the Tier-1 field's
/// capstone measured (`core/tests/capstone_neo_vs_horizons.rs`).
///
/// Each perturber pairs an [`EphemerisPerturber`] reading positions from `eph`
/// (which **must have the `sb441-n16.bsp` small-body kernel chained on**) with the
/// hardcoded DE440 GM from [`SB441_PERTURBER_GM_AU3_DAY2`], converted au³/day² →
/// km³/s² → SI. Unlike [`tier1_perturber_field`] the GM is *not* pulled from ANISE:
/// the shipped constants resolve only 6 of the 16 (and to a different, later mass
/// solution — see the table doc), so a single self-consistent DE440 set is
/// hardcoded instead.
///
/// **Fails loud if the small-body kernel is not mounted.** Positions for these NAIF
/// ids resolve only when `sb441` is chained onto `eph`; a caller that enabled the
/// asteroid perturbers but handed in a DE-only almanac would otherwise get a field
/// that fails deep inside the first integration step with an opaque lookup error.
/// This probes every body's position at a reference epoch up front and returns a
/// clear [`EphemerisError::Translate`] naming the missing kernel — the
/// "an incomplete field is a wrong field" doctrine [`tier1_perturber_field`] holds
/// for GMs, applied here to positions, since `sb441` is the optional 646 MB kernel
/// (see [`crate::kernels::KernelPair::small_bodies`]) whose absence is a real case.
pub fn sb441_perturber_field(
    eph: &Arc<Ephemeris>,
) -> Result<PointMassGravity, EphemerisError> {
    // A reference epoch well inside every relevant kernel's coverage (de440s
    // 1849–2150, sb441 1550–2650) — J2000 — at which to prove the positions
    // resolve before the field is ever handed to the integrator.
    let probe = AniseEpoch::from_gregorian(2000, 1, 1, 0, 0, 0, 0, TimeScale::TDB);

    let mut perturbers = Vec::with_capacity(SB441_PERTURBER_GM_AU3_DAY2.len());
    for &(id, name, gm_au3_day2) in &SB441_PERTURBER_GM_AU3_DAY2 {
        let frame = anise::prelude::Frame::from_ephem_j2000(id);
        // Liveness probe: a failure here means sb441 is not mounted (or lacks this
        // body), so say exactly that rather than letting the integrator fail later.
        eph.position_km(frame, SSB_J2000, probe).map_err(|e| {
            EphemerisError::Translate(format!(
                "asteroid perturber {name} ({id}) has no position — is the sb441-n16 \
                 small-body kernel mounted on this ephemeris? (underlying: {e})"
            ))
        })?;
        let mu = gm_au3_day2 * AU3_DAY2_TO_KM3_S2 * KM3_S2_TO_M3_S2;
        let source = EphemerisPerturber::new(Arc::clone(eph), frame);
        perturbers.push(Perturber::new(mu, source));
    }
    Ok(PointMassGravity::new(perturbers))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SI factors must stay distinct: positions scale by `1e3`, GM by `1e9`.
    /// Guards against the "scaled a position by the GM factor" slip the two
    /// separate constants exist to prevent.
    #[test]
    fn position_and_gm_si_factors_are_distinct() {
        assert_eq!(KM_TO_M, 1.0e3);
        assert_eq!(KM3_S2_TO_M3_S2, 1.0e9);
        assert_eq!(KM_TO_M * KM_TO_M * KM_TO_M, KM3_S2_TO_M3_S2);
    }

    /// The Tier-1 set is exactly the ten MVP bodies and never the EMB (NAIF 3),
    /// which would be the §5 geocenter footgun.
    #[test]
    fn tier1_set_is_ten_bodies_without_the_emb() {
        assert_eq!(TIER1_PERTURBER_FRAMES.len(), 10);
        use anise::constants::frames::EARTH_MOON_BARYCENTER_J2000;
        assert!(
            !TIER1_PERTURBER_FRAMES.contains(&EARTH_MOON_BARYCENTER_J2000),
            "Tier-1 field must carry the geocenter + separate Moon, not the EMB"
        );
        // Earth geocenter and the Moon are both present and distinct.
        assert!(TIER1_PERTURBER_FRAMES.contains(&EARTH_J2000));
        assert!(TIER1_PERTURBER_FRAMES.contains(&MOON_J2000));
    }

    /// Kernel-gated end-to-end: with a real DE kernel + constants set, the
    /// adapter returns a sane SSB position (in metres) and the field builder
    /// assembles all ten perturbers with positive, physically-ordered μ. Set
    /// `ASTEROID_DE_KERNEL` and `ASTEROID_PLANETARY_CONSTANTS`; skips (passes)
    /// when either is unset so CI stays green offline.
    #[test]
    fn tier1_field_builds_from_a_real_kernel() {
        let Some(k) = crate::kernels::resolve_for_test("the Tier-1 field build") else {
            return;
        };
        let (bsp, pca) = k.as_strs();
        let eph = Arc::new(
            Ephemeris::load(bsp)
                .expect("load DE kernel")
                .with_constants(pca)
                .expect("load constants"),
        );

        // Adapter: the Sun sits within a few million km of the SSB (it wobbles
        // about the barycenter under Jupiter's pull) — a sanity band that would
        // catch a km/m unit slip (off by 1e3) instantly.
        let sun = EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000);
        let epoch = Epoch::from_tdb_gregorian(2020, 1, 1, 0, 0, 0, 0);
        let r_sun = sun.position_at(epoch).expect("sun position");
        let r_sun_km = r_sun.norm() / KM_TO_M;
        assert!(
            (100_000.0..=2_000_000.0).contains(&r_sun_km),
            "Sun-SSB distance {r_sun_km:.0} km outside the expected barycentric-wobble band"
        );

        // The whole point of an *ephemeris* perturber (vs the constant
        // FixedPerturber) is that position tracks the epoch. Assert it actually
        // moves: Earth ~a quarter of the way round its orbit three months later
        // must shift by a large fraction of an AU. A silently dropped or pinned
        // epoch would leave every other check here green — this is the one that
        // wouldn't.
        let earth = EphemerisPerturber::new(Arc::clone(&eph), EARTH_J2000);
        let epoch_q2 = Epoch::from_tdb_gregorian(2020, 4, 1, 0, 0, 0, 0);
        let r_earth_jan = earth.position_at(epoch).expect("earth @ Jan");
        let r_earth_apr = earth.position_at(epoch_q2).expect("earth @ Apr");
        let swept = (r_earth_apr - r_earth_jan).norm();
        assert!(
            swept > 1.0e11,
            "Earth barely moved over 3 months ({swept:.3e} m) — epoch not threaded?"
        );

        // Field builder: ten perturbers, all μ finite and positive, Sun heaviest.
        let field = tier1_perturber_field(&eph).expect("build tier-1 field");
        assert_eq!(field.len(), 10);

        let sun_mu = eph.sun_gm_m3_s2().expect("sun gm");
        for &frame in &TIER1_PERTURBER_FRAMES {
            let mu = eph.gm_km3_s2(frame).expect("gm resolves") * KM3_S2_TO_M3_S2;
            assert!(mu > 0.0 && mu.is_finite(), "μ for {frame:?} = {mu}");
            assert!(mu <= sun_mu, "{frame:?} μ exceeds the Sun's");
        }

        // End-to-end through the actual force term: a test particle 1 AU out on
        // +x feels a total acceleration that is Sun-dominated and ~GM_sun/(1 AU)²
        // ≈ 5.9e-3 m/s² pointing sunward (−x). This exercises the whole pipeline —
        // position km→m, GM km³→m³, the 1/r² sum — so a unit slip anywhere in it
        // moves the magnitude by orders of magnitude and trips the band.
        use crate::forces::ForceModel;
        use crate::state::StateVector;
        const AU_M: f64 = 1.495_978_707e11;
        let particle = StateVector::from_components(AU_M, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a = field
            .acceleration(epoch, &particle)
            .expect("field acceleration");
        assert!(
            (5.0e-3..=7.0e-3).contains(&a.norm()),
            "|a| = {:.3e} m/s² at 1 AU is not the expected ~5.9e-3 (unit error?)",
            a.norm()
        );
        assert!(
            a.x < 0.0 && a.x.abs() > a.y.abs().max(a.z.abs()),
            "acceleration at +x 1 AU should point predominantly sunward (−x): {a:?}"
        );
    }

    /// The sb441 asteroid GM table is well-formed **without any kernel**: sixteen
    /// bodies, NAIF ids strictly ascending and on the `2000000 + number`
    /// convention, every GM finite and positive, Ceres the heaviest by a wide
    /// margin, and the au³/day² → km³/s² conversion landing each mass in a sane
    /// main-belt band (Ceres ~63, the smallest ~0.1). This is the transcription /
    /// unit guard that needs no kernel: a stray digit or a wrong conversion factor
    /// throws a body out of the band.
    #[test]
    fn sb441_gm_table_is_well_formed() {
        assert_eq!(SB441_PERTURBER_GM_AU3_DAY2.len(), 16);

        let mut prev_id = 0;
        let mut ceres_km3_s2 = 0.0;
        for &(id, name, gm) in &SB441_PERTURBER_GM_AU3_DAY2 {
            assert!(id > prev_id, "ids must be strictly ascending: {name} ({id})");
            assert!(
                (2_000_001..=2_999_999).contains(&id),
                "{name} id {id} is not on the 2000000+number convention"
            );
            prev_id = id;

            assert!(gm > 0.0 && gm.is_finite(), "GM for {name} = {gm} au³/day²");
            let km3_s2 = gm * AU3_DAY2_TO_KM3_S2;
            assert!(
                (0.01..=100.0).contains(&km3_s2),
                "{name} GM {km3_s2:.3} km³/s² is outside the main-belt band — \
                 transcription or unit error?"
            );
            if id == 2000001 {
                ceres_km3_s2 = km3_s2;
            }
        }

        // Ceres is the most massive belt asteroid; its GM must dominate.
        assert!(
            (60.0..=65.0).contains(&ceres_km3_s2),
            "Ceres GM {ceres_km3_s2:.3} km³/s² is not the expected ~62.6"
        );
        for &(id, name, gm) in &SB441_PERTURBER_GM_AU3_DAY2 {
            if id != 2000001 {
                assert!(
                    gm * AU3_DAY2_TO_KM3_S2 < ceres_km3_s2,
                    "{name} GM exceeds Ceres'"
                );
            }
        }
    }

    /// Kernel-gated end-to-end: with the sb441 small-body kernel mounted, the
    /// asteroid field builds all sixteen perturbers, and the hardcoded DE440 GMs
    /// for the three **best-determined** bodies (Ceres, Pallas, Vesta) agree with
    /// the independently-sourced pck11 SPICE constants to <1%.
    ///
    /// The three-body check is the real unit-conversion guard: a wrong au³/day²
    /// factor would miss pck11 by orders of magnitude, and Vesta in particular
    /// matches to ~4 significant figures. The other thirteen sb441 GMs are DE440's
    /// own free-fit solution and *legitimately* differ from pck11's later
    /// spacecraft/occultation determinations by tens of percent (pck11 resolves
    /// only 6 of the 16 at all) — which is exactly why the table hardcodes the
    /// self-consistent DE440 set rather than resolving GMs through ANISE. Asserting
    /// the loosely-determined bodies against pck11 would encode a disagreement that
    /// is not this code's to reconcile, so the guard deliberately checks only the
    /// three that share a determination.
    ///
    /// Skips (passes) when no sb441 kernel resolves; `ASTEROID_REQUIRE_KERNELS`
    /// turns that skip into a failure.
    #[test]
    fn sb441_field_builds_and_well_determined_gms_match_pck11() {
        let Some(k) = crate::kernels::resolve_for_test("the sb441 perturber field build") else {
            return;
        };
        let Some(sb) = k.small_bodies.clone() else {
            // The DE pair resolved but no sb441 alongside it — a real, allowed
            // configuration (sb441 is the optional 646 MB kernel), so there is
            // nothing to build here. REQUIRE_KERNELS gates the DE pair, not sb441.
            return;
        };
        let (bsp, pca) = k.as_strs();
        let eph = Arc::new(
            Ephemeris::load(bsp)
                .expect("load DE kernel")
                .with_constants(pca)
                .expect("load constants")
                .with_constants(&sb)
                .expect("mount sb441"),
        );

        let field = sb441_perturber_field(&eph).expect("build sb441 field");
        assert_eq!(field.len(), 16, "all sixteen asteroids enrolled");

        // pck11 resolves these three to the SAME determination the DE440 fit
        // adopted, so the hardcoded GM must match ANISE's to <1%.
        for &(id, name) in &[(2000001, "Ceres"), (2000002, "Pallas"), (2000004, "Vesta")] {
            let frame = anise::prelude::Frame::from_ephem_j2000(id);
            let pck_km3_s2 = eph
                .gm_km3_s2(frame)
                .unwrap_or_else(|e| panic!("pck11 GM for {name} did not resolve: {e}"));
            let hardcoded_au3_day2 = SB441_PERTURBER_GM_AU3_DAY2
                .iter()
                .find(|(fid, _, _)| *fid == id)
                .map(|(_, _, gm)| *gm)
                .expect("body in table");
            let hardcoded_km3_s2 = hardcoded_au3_day2 * AU3_DAY2_TO_KM3_S2;
            let rel = (hardcoded_km3_s2 - pck_km3_s2).abs() / pck_km3_s2;
            assert!(
                rel < 0.01,
                "{name}: hardcoded DE440 GM {hardcoded_km3_s2:.4} km³/s² vs pck11 \
                 {pck_km3_s2:.4} km³/s² differ by {:.2}% (unit or transcription error?)",
                rel * 100.0
            );
        }
    }

    /// The asteroid field **fails loud** when the sb441 kernel is not mounted:
    /// positions for the `2000000+` NAIF ids do not resolve against a DE-only
    /// almanac, so building the field must return a clear error naming the missing
    /// small-body kernel — never a field that silently omits perturbers or defers
    /// the failure to an opaque mid-integration lookup error. Kernel-gated on the
    /// DE pair only (no sb441 needed — the point is its *absence*).
    #[test]
    fn sb441_field_without_the_small_body_kernel_fails_loud() {
        let Some(k) = crate::kernels::resolve_for_test("the sb441 fail-loud check") else {
            return;
        };
        let (bsp, pca) = k.as_strs();
        // DE + constants only — deliberately no `.with_constants(sb441)`.
        let eph = Arc::new(
            Ephemeris::load(bsp)
                .expect("load DE kernel")
                .with_constants(pca)
                .expect("load constants"),
        );
        match sb441_perturber_field(&eph) {
            Err(EphemerisError::Translate(msg)) => {
                assert!(
                    msg.contains("sb441"),
                    "error should name the missing small-body kernel: {msg}"
                );
            }
            Err(other) => panic!("expected a Translate error naming sb441, got {other}"),
            Ok(_) => panic!("built an asteroid field with no small-body kernel mounted"),
        }
    }
}
