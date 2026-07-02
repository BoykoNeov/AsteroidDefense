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

use crate::ephemeris::{Ephemeris, EphemerisError, KM3_S2_TO_M3_S2};
use crate::epoch::Epoch;
use crate::forces::point_mass::{Perturber, PerturberEphemeris, PointMassGravity};
use crate::forces::ForceError;

/// SI conversion for **positions**: 1 km in metres. Separate from
/// [`KM3_S2_TO_M3_S2`] (the GM factor, `1e9`) so the two can never be confused.
pub const KM_TO_M: f64 = 1.0e3;

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
///   the barycenter mass is the standard N-body treatment (and ASSIST's — to be
///   confirmed against ASSIST in the 2c validation batch).
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
        let (Ok(bsp), Ok(pca)) = (
            std::env::var("ASTEROID_DE_KERNEL"),
            std::env::var("ASTEROID_PLANETARY_CONSTANTS"),
        ) else {
            eprintln!("ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS unset — skipping");
            return;
        };
        let eph = Arc::new(
            Ephemeris::load(&bsp)
                .expect("load DE kernel")
                .with_constants(&pca)
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
}
