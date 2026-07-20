//! Composable force model — the sum of individually-toggleable acceleration terms.
//!
//! A [`ForceModel`] answers one question: *what acceleration does the integrated
//! body feel, here and now?* (HANDOFF §4, §5). The whole realism ladder is
//! expressed as **which terms are enabled** in a [`CompositeForce`], never as
//! separate integration code paths — Tier 1 is point-mass gravity from the
//! ephemeris field; Tier 2 flips on relativity / Yarkovsky / SRP by pushing more
//! terms into the same sum (§5). Each term is unit-validated in isolation (§6).
//!
//! # Frame and units
//! Every term returns an **acceleration** (m/s², mass-independent) expressed in
//! the **barycentric (SSB) ICRF** frame, SI units — the same frame the integrator
//! advances the state in (HANDOFF §5). Acceleration, not force, because the
//! test-particle asteroid's own mass cancels out of its equation of motion; a
//! term that genuinely needs the body's mass/area (SRP, Yarkovsky) carries those
//! as its own parameters.
//!
//! # Fallibility
//! [`ForceModel::acceleration`] returns a `Result`: perturber positions come from
//! an ephemeris that can fail to resolve (ANISE, later), and a term can hit a
//! genuine singularity (coincident bodies). Failing loud with a [`ForceError`]
//! beats propagating a silent `NaN` through the integrator (matches the crate's
//! fail-loud convention).
//!
//! # Progress (§10.7)
//! [`point_mass::PointMassGravity`] is the only *term* so far. It runs on fixed
//! perturbers ([`point_mass::FixedPerturber`], kernel-free — for the integrator's
//! free-invariant tests) **and** on the real DE440/441 field via the ANISE
//! adapter in [`crate::perturber_field`] ([`crate::EphemerisPerturber`] +
//! [`crate::tier1_perturber_field`], the Sun + 8 planets + Moon MVP set). The
//! relativity/oblateness/Yarkovsky/SRP terms and the Tier-1 encounter validation
//! against ASSIST land in later batches.

pub mod point_mass;
pub mod relativity;
pub mod yarkovsky;

use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Failure modes of a force-model evaluation.
///
/// A single concrete enum (rather than a per-term associated type) keeps
/// [`ForceModel`] object-safe, so heterogeneous terms compose in one
/// [`CompositeForce`] behind `dyn ForceModel`.
#[derive(Debug, Clone, PartialEq)]
pub enum ForceError {
    /// A point-mass perturber is coincident with the integrated body, so the
    /// `1/r²` acceleration is singular. Real close approaches produce large but
    /// finite accelerations by design (that is the encounter physics); only an
    /// exactly-zero — or non-finite — separation is rejected here, since it
    /// signals a degenerate configuration, not a physical flyby.
    Singularity {
        /// Index of the offending perturber in the term's list.
        perturber_index: usize,
        /// The separation `|r_perturber − r_body|`, metres (`0` or non-finite).
        separation: f64,
    },
    /// An ephemeris lookup for a perturber position failed. Carries the
    /// underlying message; produced by the ANISE-backed perturber adapter
    /// ([`crate::EphemerisPerturber`]), never by [`point_mass::FixedPerturber`].
    Ephemeris(String),
}

impl std::fmt::Display for ForceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ForceError::Singularity {
                perturber_index,
                separation,
            } => write!(
                f,
                "point-mass singularity: perturber {perturber_index} is coincident with the body (separation = {separation:.3e} m)"
            ),
            ForceError::Ephemeris(e) => write!(f, "perturber ephemeris lookup failed: {e}"),
        }
    }
}

impl std::error::Error for ForceError {}

/// An acceleration source (HANDOFF §4). Object-safe by design: a
/// [`CompositeForce`] holds a heterogeneous list of these behind
/// `Box<dyn ForceModel>` and sums them.
///
/// **`Send + Sync` is load-bearing, not decoration.** Building a real-field
/// scenario is a ~10 s propagation, and a frontend must run it off its render
/// thread or freeze. A `Box<dyn ForceModel>` is `Send` only if the trait says so,
/// so without this the force field — and every scenario owning one — is pinned to
/// the thread that built it.
///
/// `Sync` comes along for a reason worth recording, because `Send` alone looks
/// sufficient and is not: this trait is *decorated*. A wrapper that holds another
/// model by reference (`&'a dyn ForceModel`) and implements `ForceModel` — the
/// adaptive-controller test's evaluation counter does exactly this — can only be
/// `Send` if `&dyn ForceModel` is, and `&T: Send` requires `T: Sync`. So `Send`
/// without `Sync` would quietly outlaw the decorator pattern the tests already
/// use. Every implementor is an immutable bundle of numbers and `Arc`s (the ANISE
/// almanac included), so the bound costs nothing real; it does forbid hiding
/// thread-affine mutable state (a `Cell`) behind `&self`, which is why that
/// counter is an atomic.
pub trait ForceModel: Send + Sync {
    /// Acceleration (m/s², barycentric ICRF, SI) felt by a body at `state` and
    /// `epoch`. Takes the full [`StateVector`] — velocity-dependent terms (1PN
    /// relativity, Yarkovsky, SRP shadowing) need it, even though pure point-mass
    /// gravity uses only the position.
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError>;
}

/// The force model as a **sum of toggleable terms** (HANDOFF §5). Enabling a tier
/// is pushing more terms in; disabling one is leaving it out. An empty composite
/// is a valid free particle (zero acceleration).
///
/// ```
/// use asteroid_core::forces::{CompositeForce, ForceModel};
/// use asteroid_core::forces::point_mass::{FixedPerturber, PointMassGravity};
/// use asteroid_core::{Epoch, StateVector};
/// use nalgebra::Vector3;
///
/// // μ = 1, one attractor at the origin.
/// let sun = PointMassGravity::new(vec![(1.0, FixedPerturber::at_origin()).into()]);
/// let model = CompositeForce::new().with(Box::new(sun));
/// let s = StateVector::from_components(1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
/// let a = model.acceleration(Epoch::from_tdb_seconds_past_j2000(0.0), &s).unwrap();
/// assert!((a - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-15);
/// ```
#[derive(Default)]
pub struct CompositeForce {
    terms: Vec<Box<dyn ForceModel>>,
}

impl CompositeForce {
    /// An empty force model (zero acceleration until terms are added).
    pub fn new() -> Self {
        Self { terms: Vec::new() }
    }

    /// Add a term, consuming and returning `self` for builder-style chaining.
    pub fn with(mut self, term: Box<dyn ForceModel>) -> Self {
        self.terms.push(term);
        self
    }

    /// Add a term in place.
    pub fn push(&mut self, term: Box<dyn ForceModel>) {
        self.terms.push(term);
    }

    /// Number of enabled terms.
    pub fn len(&self) -> usize {
        self.terms.len()
    }

    /// Whether no terms are enabled (a free particle).
    pub fn is_empty(&self) -> bool {
        self.terms.is_empty()
    }
}

impl ForceModel for CompositeForce {
    /// Sum the accelerations of every enabled term. Short-circuits on the first
    /// term that errors (fail-loud), rather than silently dropping a contribution.
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let mut total = Vector3::zeros();
        for term in &self.terms {
            total += term.acceleration(epoch, state)?;
        }
        Ok(total)
    }
}

#[cfg(test)]
mod tests {
    use super::point_mass::{FixedPerturber, PointMassGravity};
    use super::*;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    #[test]
    fn empty_composite_is_a_free_particle() {
        let model = CompositeForce::new();
        assert!(model.is_empty());
        let s = StateVector::from_components(3.0, -2.0, 1.0, 0.0, 0.0, 0.0);
        let a = model.acceleration(epoch0(), &s).unwrap();
        assert_eq!(a, Vector3::zeros());
    }

    #[test]
    fn composite_sums_its_terms() {
        // Two unit attractors on the ±x axis; a body at the origin feels equal and
        // opposite pulls that cancel — a sum only a composite (not one term) shows.
        let left = PointMassGravity::new(vec![(
            1.0,
            FixedPerturber::new(Vector3::new(-1.0, 0.0, 0.0)),
        )
            .into()]);
        let right = PointMassGravity::new(vec![(
            1.0,
            FixedPerturber::new(Vector3::new(1.0, 0.0, 0.0)),
        )
            .into()]);
        let model = CompositeForce::new()
            .with(Box::new(left))
            .with(Box::new(right));
        assert_eq!(model.len(), 2);

        let s = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a = model.acceleration(epoch0(), &s).unwrap();
        assert!(a.norm() < 1e-15, "opposed pulls should cancel, got {a:?}");
    }

    #[test]
    fn composite_short_circuits_on_a_term_error() {
        // One good term + one that sits on top of the body → Singularity bubbles up.
        let good = PointMassGravity::new(vec![(
            1.0,
            FixedPerturber::new(Vector3::new(10.0, 0.0, 0.0)),
        )
            .into()]);
        let coincident = PointMassGravity::new(vec![(1.0, FixedPerturber::at_origin()).into()]);
        let model = CompositeForce::new()
            .with(Box::new(good))
            .with(Box::new(coincident));
        let s = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(matches!(
            model.acceleration(epoch0(), &s),
            Err(ForceError::Singularity { .. })
        ));
    }
}
