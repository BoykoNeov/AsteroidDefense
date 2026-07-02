//! Point-mass gravity from an arbitrary list of perturbers (HANDOFF §5, §10.7).
//!
//! The Tier-1 MVP integrates the asteroid as a **test particle** in a field of
//! point-mass perturbers — Sun + all planets + Moon, positions and GM from the
//! DE440/441 ephemeris via ANISE. This term computes exactly that acceleration:
//! for a body at `r`, each perturber `j` at `r_j` with gravitational parameter
//! `μ_j` contributes
//!
//! ```text
//! a_j = μ_j · (r_j − r) / |r_j − r|³ ,   a = Σ_j a_j
//! ```
//!
//! The list is deliberately open-ended: the same term carries one attractor (the
//! two-body tests below), the nine MVP bodies, or the +16 major-asteroid
//! perturbers at Tier 2 — adding bodies is a data change, never a code change
//! (§5). Perturber positions come through the [`PerturberEphemeris`] trait, so
//! the term is decoupled from *where* the positions originate: [`FixedPerturber`]
//! (constant, for tests and a fixed attractor) now; an ANISE [`Ephemeris`]
//! adapter later.
//!
//! # Frame
//! [`PerturberEphemeris::position_at`] returns a position in the **barycentric
//! (SSB) ICRF** frame, SI metres — the integration frame (HANDOFF §5). This is a
//! *different contract* from [`crate::propagator::Propagator`], whose output is
//! attractor-relative; conflating the two is the §5 frame footgun, so perturber
//! positions get their own frame-explicit trait rather than reusing `Propagator`.
//!
//! [`Ephemeris`]: crate::ephemeris::Ephemeris

use super::{ForceError, ForceModel};
use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Source of a perturber's **position** at an epoch, in the barycentric (SSB)
/// ICRF frame, SI metres (HANDOFF §5).
///
/// Frame-explicit and position-only by design. It is *not*
/// [`crate::propagator::Propagator`]: that trait's state is attractor-relative,
/// whereas a perturber position must be in the barycentric integration frame.
/// A `velocity_at` sibling will join this when a velocity-dependent term (1PN,
/// SRP) needs it; omitted now (YAGNI — pure point-mass gravity needs only
/// position).
pub trait PerturberEphemeris {
    /// Position of the perturber at `epoch`, metres, barycentric ICRF.
    fn position_at(&self, epoch: Epoch) -> Result<Vector3<f64>, ForceError>;
}

/// A perturber whose position never changes — a fixed point in the frame.
///
/// The workhorse of the RK4-first batch: a single `FixedPerturber::at_origin()`
/// with a heliocentric μ turns [`PointMassGravity`] into a clean two-body field
/// whose analytic solution ([`crate::propagator::KeplerPropagator`]) is a valid
/// oracle *because the attractor sits at the frame origin* — so the propagator's
/// attractor-relative state coincides with the barycentric state (HANDOFF §5).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedPerturber {
    position: Vector3<f64>,
}

impl FixedPerturber {
    /// A perturber pinned at `position` (metres, barycentric ICRF).
    pub fn new(position: Vector3<f64>) -> Self {
        Self { position }
    }

    /// A perturber pinned at the frame origin.
    pub fn at_origin() -> Self {
        Self {
            position: Vector3::zeros(),
        }
    }
}

impl PerturberEphemeris for FixedPerturber {
    fn position_at(&self, _epoch: Epoch) -> Result<Vector3<f64>, ForceError> {
        Ok(self.position)
    }
}

/// A single gravitating perturber: its gravitational parameter `μ` (m³/s²) and a
/// source for its position over time.
pub struct Perturber {
    /// Gravitational parameter `μ = GM`, SI (m³/s²).
    pub mu: f64,
    /// Where this perturber is at any epoch (barycentric ICRF, SI).
    pub ephemeris: Box<dyn PerturberEphemeris>,
}

impl Perturber {
    /// Build a perturber from a `μ` and any [`PerturberEphemeris`].
    pub fn new(mu: f64, ephemeris: impl PerturberEphemeris + 'static) -> Self {
        Self {
            mu,
            ephemeris: Box::new(ephemeris),
        }
    }
}

impl<E: PerturberEphemeris + 'static> From<(f64, E)> for Perturber {
    /// `(μ, ephemeris).into()` — the ergonomic constructor used to build a
    /// perturber list inline.
    fn from((mu, ephemeris): (f64, E)) -> Self {
        Perturber::new(mu, ephemeris)
    }
}

/// Newtonian point-mass gravity from a list of [`Perturber`]s acting on a test
/// particle (HANDOFF §5). The particle's own mass cancels, so this returns an
/// acceleration; the particle exerts no back-reaction on the perturbers (they
/// follow their own ephemeris).
pub struct PointMassGravity {
    perturbers: Vec<Perturber>,
}

impl PointMassGravity {
    /// Build the term from a perturber list. An empty list is a valid (zero)
    /// field — the composite decides whether that is meaningful.
    pub fn new(perturbers: Vec<Perturber>) -> Self {
        Self { perturbers }
    }

    /// Number of perturbers in the field.
    pub fn len(&self) -> usize {
        self.perturbers.len()
    }

    /// Whether the field has no perturbers.
    pub fn is_empty(&self) -> bool {
        self.perturbers.is_empty()
    }
}

impl ForceModel for PointMassGravity {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let r = state.position;
        let mut total = Vector3::zeros();
        for (index, perturber) in self.perturbers.iter().enumerate() {
            let r_j = perturber.ephemeris.position_at(epoch)?;
            let d = r_j - r;
            let dist = d.norm();
            // Fail loud on an exactly-zero or non-finite separation (a degenerate
            // coincidence). A real close approach is a small-but-positive `dist`
            // and passes through as a large finite acceleration (the encounter
            // physics); only `0` or `NaN` (a NaN input position) is rejected.
            if dist == 0.0 || !dist.is_finite() {
                return Err(ForceError::Singularity {
                    perturber_index: index,
                    separation: dist,
                });
            }
            total += perturber.mu * d / (dist * dist * dist);
        }
        Ok(total)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    #[test]
    fn single_attractor_matches_the_closed_form() {
        // One attractor of μ at the origin; a body at distance r on +x feels
        // a = −μ/r² x̂ (points back toward the attractor).
        let mu = 3.5;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let r = 2.0;
        let s = StateVector::from_components(r, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a = field.acceleration(epoch0(), &s).unwrap();
        assert!(
            (a - Vector3::new(-mu / (r * r), 0.0, 0.0)).norm() < 1e-15,
            "a = {a:?}"
        );
    }

    #[test]
    fn acceleration_points_from_body_toward_perturber() {
        // Off-axis perturber: the acceleration is along (r_j − r), magnitude μ/d².
        let mu = 7.0;
        let r_j = Vector3::new(4.0, 3.0, 0.0); // |r_j| = 5
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::new(r_j)).into()]);
        let s = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a = field.acceleration(epoch0(), &s).unwrap();
        let expected = mu / (5.0 * 5.0) * r_j.normalize();
        assert!(
            (a - expected).norm() < 1e-14,
            "a = {a:?}, expected {expected:?}"
        );
    }

    #[test]
    fn two_perturbers_superpose_linearly() {
        // The field is a plain sum; check against a hand-summed expectation.
        let a_pert = (2.0, FixedPerturber::new(Vector3::new(1.0, 0.0, 0.0)));
        let b_pert = (5.0, FixedPerturber::new(Vector3::new(0.0, -4.0, 0.0)));
        let s = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        let both = PointMassGravity::new(vec![a_pert.into(), b_pert.into()]);
        let a_only = PointMassGravity::new(vec![a_pert.into()]);
        let b_only = PointMassGravity::new(vec![b_pert.into()]);

        let sum =
            a_only.acceleration(epoch0(), &s).unwrap() + b_only.acceleration(epoch0(), &s).unwrap();
        let together = both.acceleration(epoch0(), &s).unwrap();
        assert!((together - sum).norm() < 1e-15);
    }

    #[test]
    fn coincident_perturber_fails_loud() {
        let field = PointMassGravity::new(vec![
            (1.0, FixedPerturber::new(Vector3::new(1.0, 0.0, 0.0))).into(),
            (1.0, FixedPerturber::new(Vector3::new(2.0, 2.0, 2.0))).into(),
        ]);
        // Body sits exactly on the second perturber.
        let s = StateVector::from_components(2.0, 2.0, 2.0, 0.0, 0.0, 0.0);
        match field.acceleration(epoch0(), &s) {
            Err(ForceError::Singularity {
                perturber_index,
                separation,
            }) => {
                assert_eq!(perturber_index, 1);
                assert_eq!(separation, 0.0);
            }
            other => panic!("expected Singularity, got {other:?}"),
        }
    }

    #[test]
    fn empty_field_is_zero_acceleration() {
        let field = PointMassGravity::new(vec![]);
        assert!(field.is_empty());
        let s = StateVector::from_components(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
        assert_eq!(field.acceleration(epoch0(), &s).unwrap(), Vector3::zeros());
    }
}
