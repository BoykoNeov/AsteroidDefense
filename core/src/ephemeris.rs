//! Ephemeris loading — the entry point for perturber positions.
//!
//! The Tier-1 MVP integrates the asteroid as a *test particle* in the DE440/441
//! ephemeris field, so the core needs perturber **positions** (not just GM
//! constants) from day one. This module loads a local SPICE/DE kernel via ANISE
//! and hands back SSB-relative positions in the barycentric ICRF frame.
//!
//! **The DE440/441 geocenter footgun (HANDOFF §5).** A JPL DE kernel natively
//! stores the Earth–Moon *barycenter* (EMB, NAIF id 3) as an SSB-relative
//! segment, plus separate EMB→Earth (399) and EMB→Moon (301) segments. Using
//! the EMB as "Earth's position" displaces Earth by ~4671 km — an
//! Earth-radius-scale error that corrupts the b-plane. ANISE reconstructs the
//! true geocenter by walking SSB→EMB→Earth; [`Ephemeris::geocenter_ssb_km`]
//! returns that reconstructed geocenter, **not** the EMB. The task-0.5 de-risk
//! spike ([`verify_geocenter_reconstruction`]) proves this empirically.

use anise::constants::frames::{
    EARTH_J2000, EARTH_MOON_BARYCENTER_J2000, MOON_J2000, SSB_J2000, SUN_J2000,
};
use anise::math::Vector3;
use anise::prelude::{Almanac, Epoch, Frame};

/// SI conversion: 1 km³/s² in m³/s². ANISE carries gravitational parameters in
/// km³/s²; the core physics runs in SI, so GM crosses the boundary through this
/// factor (`(1e3 m)³ / s² = 1e9 m³/s²`).
pub const KM3_S2_TO_M3_S2: f64 = 1.0e9;

use std::path::{Path, PathBuf};

/// DE440/DE441 Earth-to-Moon mass ratio (EMRAT), i.e. `M_earth / M_moon`.
///
/// Used only by the de-risk spike to *independently* reconstruct the EMB from
/// the separate Earth and Moon geocenters and confirm ANISE's Earth segment is
/// the genuine geocenter. Value from the DE440/441 header constants.
///
/// The literal carries more digits than an `f64` can distinguish (the last two
/// round away), but they are the *verbatim* published header constant, kept for
/// provenance — clippy confirms the truncation would be bit-identical, so the
/// stored value is unchanged either way.
#[allow(clippy::excessive_precision)]
pub const DE440_EMRAT: f64 = 81.300_568_221_497_215_4;

/// Errors that can arise while loading or querying an ephemeris kernel.
#[derive(Debug)]
pub enum EphemerisError {
    /// The requested kernel path does not exist on disk.
    NotFound(PathBuf),
    /// ANISE failed to load or parse the kernel.
    Load(String),
    /// ANISE failed to translate between two frames at the requested epoch.
    Translate(String),
    /// ANISE could not resolve a gravitational parameter for the requested frame
    /// (no planetary-constants set loaded, or it does not populate this body's GM).
    Constants(String),
}

impl std::fmt::Display for EphemerisError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EphemerisError::NotFound(p) => {
                write!(f, "ephemeris kernel not found: {}", p.display())
            }
            EphemerisError::Load(e) => write!(f, "ephemeris load failed: {e}"),
            EphemerisError::Translate(e) => write!(f, "ephemeris translate failed: {e}"),
            EphemerisError::Constants(e) => write!(f, "gravitational parameter lookup failed: {e}"),
        }
    }
}

impl std::error::Error for EphemerisError {}

/// Handle to a loaded ephemeris, wrapping an ANISE [`Almanac`].
///
/// All accessors return positions in **kilometres**, expressed in the
/// SSB-centered ICRF (J2000) frame — the barycentric integration frame the core
/// works in (HANDOFF §5). Callers convert to SI metres at the core boundary.
pub struct Ephemeris {
    almanac: Almanac,
    kernel_path: PathBuf,
}

impl Ephemeris {
    /// Load a DE kernel (e.g. `de440s.bsp`) from a local path. Offline only —
    /// ANISE reads the bytes from disk; nothing is auto-downloaded. Validates
    /// the path exists first so callers fail loudly rather than inside ANISE.
    pub fn load(kernel_path: impl AsRef<Path>) -> Result<Self, EphemerisError> {
        let kernel_path = kernel_path.as_ref().to_path_buf();
        if !kernel_path.exists() {
            return Err(EphemerisError::NotFound(kernel_path));
        }
        let path_str = kernel_path.to_string_lossy();
        let almanac = Almanac::new(&path_str).map_err(|e| EphemerisError::Load(e.to_string()))?;
        Ok(Self {
            almanac,
            kernel_path,
        })
    }

    /// Path of the kernel this handle was created for.
    pub fn kernel_path(&self) -> &Path {
        &self.kernel_path
    }

    /// Load an additional file — a planetary-constants set (`.pca`) carrying GM /
    /// shape data — into this handle's almanac, so [`gm_km3_s2`](Self::gm_km3_s2)
    /// lookups resolve. Builder: `Ephemeris::load(bsp)?.with_constants(pca)?`.
    ///
    /// A DE `.bsp` supplies perturber *positions* but no gravitational
    /// parameters; those come from a separate constants set (HANDOFF §6, "pull GM
    /// through ANISE"). Offline only — validates the path exists first, matching
    /// [`load`](Self::load).
    pub fn with_constants(self, constants_path: impl AsRef<Path>) -> Result<Self, EphemerisError> {
        let constants_path = constants_path.as_ref();
        if !constants_path.exists() {
            return Err(EphemerisError::NotFound(constants_path.to_path_buf()));
        }
        let path_str = constants_path.to_string_lossy();
        let Ephemeris {
            almanac,
            kernel_path,
        } = self;
        let almanac = almanac
            .load(&path_str)
            .map_err(|e| EphemerisError::Load(e.to_string()))?;
        Ok(Ephemeris {
            almanac,
            kernel_path,
        })
    }

    /// Gravitational parameter μ of `frame`, in **km³/s²**, as carried by a loaded
    /// planetary-constants set. Errors with [`EphemerisError::Constants`] if no
    /// constants populate this frame's GM (e.g. only a `.bsp` was loaded).
    ///
    /// The core physics runs in SI; multiply by [`KM3_S2_TO_M3_S2`] at the
    /// boundary, or use [`sun_gm_m3_s2`](Self::sun_gm_m3_s2).
    pub fn gm_km3_s2(&self, frame: Frame) -> Result<f64, EphemerisError> {
        let resolved = self
            .almanac
            .frame_info(frame)
            .map_err(|e| EphemerisError::Constants(e.to_string()))?;
        resolved
            .mu_km3_s2()
            .map_err(|e| EphemerisError::Constants(e.to_string()))
    }

    /// Sun gravitational parameter μ in **SI** (m³/s²) — the heliocentric μ the
    /// two-body/analytic-Kepler layer and the validation fixtures pin against,
    /// pulled through ANISE rather than hard-coded (HANDOFF §6).
    pub fn sun_gm_m3_s2(&self) -> Result<f64, EphemerisError> {
        Ok(self.gm_km3_s2(SUN_J2000)? * KM3_S2_TO_M3_S2)
    }

    /// Position **and velocity** of `target` relative to `observer` at `epoch`,
    /// as `(radius_km, velocity_km_s)`, ICRF.
    ///
    /// Geometric (no aberration correction) — the integration wants true
    /// geometric states, not light-time/stellar-aberration-corrected ones. The
    /// velocity is the component ANISE already computes alongside the radius
    /// (`translate` returns both); the close-approach detector needs it to form
    /// the Earth-**relative** velocity `v_rel` the b-plane geometry consumes
    /// (HANDOFF §10.8 step-9 note, §10.9).
    pub fn state_km_s(
        &self,
        target: Frame,
        observer: Frame,
        epoch: Epoch,
    ) -> Result<(Vector3, Vector3), EphemerisError> {
        let state = self
            .almanac
            .translate(target, observer, epoch, None)
            .map_err(|e| EphemerisError::Translate(e.to_string()))?;
        Ok((state.radius_km, state.velocity_km_s))
    }

    /// Position of `target` relative to `observer` at `epoch`, in km, ICRF —
    /// the position half of [`state_km_s`](Self::state_km_s).
    pub fn position_km(
        &self,
        target: Frame,
        observer: Frame,
        epoch: Epoch,
    ) -> Result<Vector3, EphemerisError> {
        Ok(self.state_km_s(target, observer, epoch)?.0)
    }

    /// SSB→**geocenter** (reconstructed Earth, NAIF 399), km. NOT the EMB.
    pub fn geocenter_ssb_km(&self, epoch: Epoch) -> Result<Vector3, EphemerisError> {
        self.position_km(EARTH_J2000, SSB_J2000, epoch)
    }

    /// SSB→**geocenter** state (reconstructed Earth, NAIF 399) as
    /// `(radius_km, velocity_km_s)`, ICRF. NOT the EMB. The Earth state the
    /// close-approach detector differences the asteroid track against to form the
    /// Earth-relative encounter state (§10.9).
    pub fn geocenter_state_ssb_km(
        &self,
        epoch: Epoch,
    ) -> Result<(Vector3, Vector3), EphemerisError> {
        self.state_km_s(EARTH_J2000, SSB_J2000, epoch)
    }

    /// SSB→Earth–Moon barycenter (NAIF 3), km. Provided so callers can compare
    /// against the geocenter; do NOT use this as Earth's position (footgun).
    pub fn emb_ssb_km(&self, epoch: Epoch) -> Result<Vector3, EphemerisError> {
        self.position_km(EARTH_MOON_BARYCENTER_J2000, SSB_J2000, epoch)
    }

    /// SSB→Moon (NAIF 301), km. The Moon is carried as a separate perturber
    /// through the encounter (HANDOFF §5), never lumped into the EMB.
    pub fn moon_ssb_km(&self, epoch: Epoch) -> Result<Vector3, EphemerisError> {
        self.position_km(MOON_J2000, SSB_J2000, epoch)
    }

    /// SSB→Sun (NAIF 10), km.
    pub fn sun_ssb_km(&self, epoch: Epoch) -> Result<Vector3, EphemerisError> {
        self.position_km(SUN_J2000, SSB_J2000, epoch)
    }
}

/// Outcome of the task-0.5 geocenter-reconstruction spike at one epoch.
#[derive(Debug, Clone)]
pub struct GeocenterCheck {
    /// The epoch the check was run at.
    pub epoch: Epoch,
    /// Reconstructed geocenter (Earth 399) relative to SSB, km.
    pub geocenter_ssb_km: Vector3,
    /// EMB (3) relative to SSB, km.
    pub emb_ssb_km: Vector3,
    /// Moon (301) relative to SSB, km.
    pub moon_ssb_km: Vector3,
    /// Distance between the reconstructed geocenter and the EMB, km. Expected
    /// ~4671 km; a value near 0 means ANISE handed back the EMB (footgun hit).
    pub geocenter_emb_offset_km: f64,
    /// Earth–Moon distance, km (sanity: ~356k–406k km).
    pub earth_moon_distance_km: f64,
    /// Residual between ANISE's EMB and an EMB independently reconstructed from
    /// the Earth and Moon geocenters via [`DE440_EMRAT`], km. Near 0 (< 1 km)
    /// proves ANISE's Earth segment is the true geocenter, self-consistent with
    /// the EMB and Moon — not a relabelled EMB.
    pub emrat_residual_km: f64,
}

impl GeocenterCheck {
    /// Whether this check passes the spike's acceptance criteria:
    /// geocenter distinct from the EMB by a plausible ~4671 km, a physical
    /// Earth–Moon distance, and EMRAT self-consistency to sub-km.
    pub fn passes(&self) -> bool {
        // Earth–Moon distance band brackets the true perigee/apogee range
        // (~356 500–406 700 km) with a small margin so a change of test epoch
        // can't clip it; the offset band is that distance / (EMRAT+1).
        (4200.0..=5000.0).contains(&self.geocenter_emb_offset_km)
            && (355_000.0..=407_500.0).contains(&self.earth_moon_distance_km)
            && self.emrat_residual_km < 1.0
    }
}

/// Task-0.5 de-risk spike (pillar b): confirm the ANISE DE-position reader
/// returns a **reconstructed geocenter**, not the EMB, at a known epoch.
///
/// Computes the geocenter, EMB, and Moon positions from `eph`, then checks:
/// 1. geocenter ≠ EMB, offset ~4671 km (the footgun-avoidance signal);
/// 2. Earth–Moon distance is physical;
/// 3. an EMB *independently* reconstructed from Earth+Moon via [`DE440_EMRAT`]
///    matches ANISE's EMB to sub-km — proving self-consistency.
pub fn verify_geocenter_reconstruction(
    eph: &Ephemeris,
    epoch: Epoch,
) -> Result<GeocenterCheck, EphemerisError> {
    let geocenter = eph.geocenter_ssb_km(epoch)?;
    let emb = eph.emb_ssb_km(epoch)?;
    let moon = eph.moon_ssb_km(epoch)?;

    // Independently reconstruct the EMB: EMB = (EMRAT*r_earth + r_moon)/(EMRAT+1).
    let emb_from_bodies = (geocenter * DE440_EMRAT + moon) / (DE440_EMRAT + 1.0);

    Ok(GeocenterCheck {
        epoch,
        geocenter_ssb_km: geocenter,
        emb_ssb_km: emb,
        moon_ssb_km: moon,
        geocenter_emb_offset_km: (geocenter - emb).norm(),
        earth_moon_distance_km: (moon - geocenter).norm(),
        emrat_residual_km: (emb - emb_from_bodies).norm(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use anise::time::TimeScale;

    #[test]
    fn missing_kernel_reports_not_found() {
        // Not `.unwrap_err()`: the Ok variant (Ephemeris) wraps a non-Debug
        // ANISE Almanac, so match on the error directly instead.
        match Ephemeris::load("does/not/exist/de440.bsp") {
            Err(EphemerisError::NotFound(_)) => {}
            Err(e) => panic!("expected NotFound, got {e}"),
            Ok(_) => panic!("expected NotFound error, got Ok"),
        }
    }

    /// Task-0.5 spike, gated on a real DE kernel. Set `ASTEROID_DE_KERNEL` to a
    /// local `de440s.bsp` (or DE440/441) to run it; the test skips (passes)
    /// when the env var is unset so the suite stays green offline in CI.
    #[test]
    fn geocenter_is_reconstructed_not_emb() {
        let Ok(kernel) = std::env::var("ASTEROID_DE_KERNEL") else {
            eprintln!("ASTEROID_DE_KERNEL unset — skipping geocenter spike test");
            return;
        };
        let eph = Ephemeris::load(&kernel).expect("load kernel");
        // A known epoch well inside the de440s span (1849–2150).
        let epoch = Epoch::from_gregorian(2020, 1, 1, 0, 0, 0, 0, TimeScale::TDB);
        let check = verify_geocenter_reconstruction(&eph, epoch).expect("geocenter check");

        assert!(
            check.geocenter_emb_offset_km > 1.0,
            "geocenter equals the EMB (offset {:.3} km) — footgun hit",
            check.geocenter_emb_offset_km
        );
        assert!(
            check.passes(),
            "geocenter check failed acceptance: {check:#?}"
        );
    }

    /// Kernel-gated: the surfaced geocenter **velocity** is Earth's real orbital
    /// speed (~29.8 km/s about the SSB), not garbage or zero. Pins the velocity
    /// half of [`state_km_s`] the close-approach detector's `v_rel` rides on —
    /// a silently-dropped or wrong-units velocity would fail this band. Skips
    /// (passes) when `ASTEROID_DE_KERNEL` is unset so CI stays green offline.
    #[test]
    fn geocenter_velocity_is_earth_orbital_speed() {
        let Ok(kernel) = std::env::var("ASTEROID_DE_KERNEL") else {
            eprintln!("ASTEROID_DE_KERNEL unset — skipping geocenter velocity test");
            return;
        };
        let eph = Ephemeris::load(&kernel).expect("load kernel");
        let epoch = Epoch::from_gregorian(2020, 1, 1, 0, 0, 0, 0, TimeScale::TDB);
        let (_r, v_km_s) = eph.geocenter_state_ssb_km(epoch).expect("geocenter state");
        let speed = v_km_s.norm();
        // Earth's heliocentric speed varies ~29.3–30.3 km/s over the year; the SSB
        // frame adds only the Sun's small barycentric wobble. A generous band.
        assert!(
            (29.0..=30.5).contains(&speed),
            "geocenter SSB speed {speed:.3} km/s is not Earth's ~29.8 km/s orbital speed"
        );
    }
}
