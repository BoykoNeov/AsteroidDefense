//! `J2` oblateness — the last Tier-2 force term (HANDOFF §5, §6, §166).
//!
//! A planet is not a point mass. Earth's rotation flattens it, moving mass from
//! the poles toward the equator, and the leading correction to the resulting
//! gravity field is the **`J2` zonal harmonic** — a term two orders of magnitude
//! larger than every other harmonic. It is deliberately the *last* Tier-2 term
//! this project shipped, and the note explaining why is worth keeping: `J2` falls
//! off as `1/r⁴` against point-mass gravity's `1/r²`, so heliocentrically it is
//! nothing (the Sun's `J2 ≈ 2.2e-7` at 1 AU is unmeasurably small next to the
//! terms already carried). It earns its place for exactly one regime — a **very
//! close Earth flyby**, where the asteroid spends minutes inside a few Earth
//! radii and the oblate field is the difference between one b-plane perigee and
//! another. That is keyhole territory (Tier 3), and this is the term that makes
//! the geometry there honest.
//!
//! # The acceleration
//! With `r` the body's position **relative to the oblate planet's centre**,
//! `k̂` the planet's spin axis (north pole) as a unit vector in the integration
//! frame, and `s = r̂·k̂` the sine of the body's latitude:
//!
//! ```text
//! a = −(3/2) · J2 · μ · R_eq² / r⁴ · [ (1 − 5 s²) · r̂ + 2 s · k̂ ]
//! ```
//!
//! The signs are the whole content of the term, and both are pinned by the
//! isolation tests: over the **equator** (`s = 0`) the acceleration is radially
//! **inward** — the extra equatorial mass pulls harder than a point mass would —
//! while over the **pole** (`s = 1`) it is radially **outward** by twice the
//! magnitude. Between them lies the "magic latitude" `sin φ = 1/√5` (≈ 26.57°),
//! where the bracket's `r̂` coefficient vanishes and the acceleration is purely
//! **anti-parallel to the spin axis** — pointing due south in the northern
//! hemisphere. (It still has a radial *projection* there, since `k̂` itself is not
//! perpendicular to `r̂`; what vanishes is the `r̂` term, not `a·r̂`.) A sign error
//! anywhere in the bracket breaks at least one of those three.
//!
//! # Why the pole is a parameter and not `ẑ`
//! `J2` is defined about the planet's **spin axis**, not about the integration
//! frame's `z`. For Earth in ICRF the two are close — the IAU pole sits ~0.2° off
//! ICRF north over this project's epochs — but "close" is how a display-grade
//! shortcut gets in, so the axis is supplied by a [`BodyPole`] provider rather
//! than assumed. [`FixedPole`] keeps the isolation tests kernel-free; the shipping
//! wiring hands in the pole ANISE rotates out of the loaded planetary constants,
//! so the term's axis and the frame's orientation data are the same source.
//!
//! # `J2` and `R_eq` are a *pair*, never mixed sources
//! `J2` is dimensionless only relative to a stated reference radius: the product
//! `J2 · R_eq²` is what the physics contains, so a `J2` from one solution used
//! with an `R_eq` from another is a silent scale error. [`EARTH_J2_DE440`] and
//! [`EARTH_EQUATORIAL_RADIUS_M_DE440`] are therefore transcribed **together** from
//! the same DE440 header (`J2E`, `RE`), read straight out of the local
//! `linux_p1550p2650.440` binary's constant record — the same machine-verified
//! provenance path the sb441 asteroid GMs took, and the same reason: these are the
//! values JPL *integrated the ephemeris with*.
//!
//! Note this makes [`EARTH_EQUATORIAL_RADIUS_M_DE440`] (6 378 136.6 m) a
//! deliberately *different* constant from
//! [`EARTH_EQUATORIAL_RADIUS_M`](crate::geometry::EARTH_EQUATORIAL_RADIUS_M)
//! (WGS-84, 6 378 137.0 m). They differ by 0.4 m and they are not
//! interchangeable: the WGS-84 figure is a *target radius* for the hit test, this
//! one is the reference radius `J2E` is defined against. Keeping them separate is
//! the pairing rule above, not duplication.
//!
//! # Validity boundary — and it is NOT harmless here (measured)
//! The `J2` expansion solves Laplace's equation **outside** the body and diverges
//! from reality inside `R_eq`, where the true field tends toward a solid-body
//! interior solution. This term evaluates the same formula everywhere, which only
//! arises for a trajectory that passes below the surface — the shipping scenario's
//! nominal *impact*, whose closest approach is 3000 km from Earth's centre, well
//! inside `R_eq`.
//!
//! An earlier version of this note claimed that was harmless, on the reasoning that
//! nothing downstream reads the sub-surface arc. **That was wrong, and measuring it
//! is what showed it.** The b-plane reduction samples the encounter state *at*
//! closest approach and infers the hyperbolic excess from the **point-mass** energy
//! `v_∞² = v² − 2μ/r`. At `r = 3000 km` the `J2` correction to the potential is
//! ~`J2·(R_eq/r)² ≈ 4.9e-3` of `μ/r`, and since `2μ/r` is a large fraction of `v²`
//! there, that leverages into a ~1% shift in the inferred `v_∞` — which shows up
//! as the **capture radius moving 11 311.3 → 11 389.0 km** with `J2` on, a 78 km
//! (0.69%) change, against a perigee shift of only 1.33 km.
//!
//! The control that identifies the mechanism: 1PN relativity also perturbs the
//! potential but leaves the capture radius at 11 311.3 km to the digit — its
//! correction at that radius is ~1e-9 relative, against `J2`'s ~5e-3. So this is
//! `J2`'s `1/r⁴` growth inside the body, not something generic about the b-plane
//! reduction.
//!
//! **What that means for reading the numbers.** For any geometry whose perigee lies
//! *outside* `R_eq` — every deflected, missing trajectory, which is the case the
//! project actually cares about — the term is inside its valid domain and this does
//! not arise. For the designed sub-surface impact it does, so the quoted 1.33 km
//! `J2` perigee shift should be read as "of order a kilometre, on a geometry that
//! grazes the model's validity boundary", not as a clean number. Measuring the term
//! on a genuine miss geometry is the honest follow-up.
//!
//! # Kernel-free by construction
//! Pure geometry over a caller-supplied `μ`, `J2`, `R_eq`, planet state and pole —
//! no ephemeris of its own, exactly like [`super::relativity`] and [`super::srp`].
//! Validated in isolation against the closed-form **nodal regression**
//! `dΩ/dt = −(3/2)·n·J2·(R/p)²·cos i`, with a `J2 = 0` control run and an explicit
//! inclination sign pair.

use super::relativity::CentralBodyState;
use super::{ForceError, ForceModel};
use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Earth's dynamical `J2`, DE440/441 header constant `J2E`.
///
/// Read verbatim from the constant record of the local `linux_p1550p2650.440`
/// binary (name/value arrays aligned and pinned by the header's own `AU`, `EMRAT`
/// and `DENUM` reappearing at their named slots), not recalled and not taken from
/// a documentation page. Pairs with [`EARTH_EQUATORIAL_RADIUS_M_DE440`] — see the
/// module note on why the pair must travel together.
///
/// DE440 also carries `J2EDOT = −5.9e-12`/century: `J2` drifts as Earth's mass
/// redistributes. Over this project's decades that is a ~1e-13 change on a 1.08e-3
/// constant (a relative 1e-10), far below anything the b-plane resolves, so the
/// static value is used and the drift is noted rather than modelled.
pub const EARTH_J2_DE440: f64 = 0.001_082_625_39;

/// Earth's equatorial reference radius in **metres**, DE440/441 header constant
/// `RE` (6378.1366 km).
///
/// This is the radius [`EARTH_J2_DE440`] is defined against, and the two are only
/// meaningful together (the physics carries `J2 · R_eq²`). Deliberately distinct
/// from the WGS-84
/// [`EARTH_EQUATORIAL_RADIUS_M`](crate::geometry::EARTH_EQUATORIAL_RADIUS_M) the
/// hit test uses as a target radius; see the module note.
pub const EARTH_EQUATORIAL_RADIUS_M_DE440: f64 = 6_378_136.6;

/// The orientation half of an oblate body: where its **spin axis** points.
///
/// Separated from [`CentralBodyState`] (which answers *where the body is*) because
/// the two come from different data — position from an ephemeris segment,
/// orientation from planetary constants — and because the isolation tests want a
/// fixed axis with a moving body, or vice versa. `Send + Sync` for the same
/// thread-mobility reason as [`ForceModel`]: the term lives inside a force field
/// that leaves the render thread for the scenario build.
pub trait BodyPole: Send + Sync {
    /// Unit vector along the body's north spin axis at `epoch`, expressed in the
    /// **integration frame** (barycentric ICRF). Implementations must return a
    /// unit-length vector; [`Oblateness`] normalises defensively regardless, since
    /// a silently un-normalised axis would rescale the whole term.
    fn pole_at(&self, epoch: Epoch) -> Result<Vector3<f64>, ForceError>;
}

/// A spin axis that never moves — the kernel-free configuration the isolation
/// tests use, and a legitimate approximation for any body whose precession is
/// negligible over the arc in question.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedPole {
    axis: Vector3<f64>,
}

impl FixedPole {
    /// A pole along `axis` (normalised on construction; a zero vector is rejected
    /// at use time via [`ForceError::Singularity`] rather than panicking here).
    pub fn new(axis: Vector3<f64>) -> Self {
        Self { axis }
    }

    /// The frame's `+z` axis — celestial north for an ICRF frame.
    pub fn frame_z() -> Self {
        Self::new(Vector3::z())
    }
}

impl BodyPole for FixedPole {
    fn pole_at(&self, _epoch: Epoch) -> Result<Vector3<f64>, ForceError> {
        Ok(self.axis)
    }
}

/// The `J2` zonal-harmonic acceleration of an oblate central body (HANDOFF §5).
///
/// Holds the body's `μ` (m³/s²), its dimensionless `J2`, the reference equatorial
/// radius `R_eq` (m) that `J2` is defined against, a [`CentralBodyState`] for the
/// body's position, and a [`BodyPole`] for its spin axis. Like every other term
/// the integrated body is a test particle — its own mass cancels — so this returns
/// an acceleration, and the central body follows its own ephemeris with no
/// back-reaction.
pub struct Oblateness {
    /// Gravitational parameter `μ = GM` of the oblate body, SI (m³/s²).
    mu: f64,
    /// Dimensionless `J2` coefficient, defined against `r_eq`.
    j2: f64,
    /// Reference equatorial radius `J2` is normalised to, metres.
    r_eq: f64,
    /// Where the oblate body is at any epoch (only its position is read).
    central: Box<dyn CentralBodyState>,
    /// Which way its spin axis points at any epoch.
    pole: Box<dyn BodyPole>,
}

impl Oblateness {
    /// Build the term from the body's `μ`, its `J2`/`R_eq` pair, a position source
    /// and a pole source.
    pub fn new(
        mu: f64,
        j2: f64,
        r_eq: f64,
        central: impl CentralBodyState + 'static,
        pole: impl BodyPole + 'static,
    ) -> Self {
        Self {
            mu,
            j2,
            r_eq,
            central: Box::new(central),
            pole: Box::new(pole),
        }
    }

    /// Earth's oblateness with the DE440 `J2E`/`RE` pair, given `μ⊕` (pull it
    /// through ANISE, never a second hardcoded constant — the same rule the 1PN
    /// term follows for `μ_sun`) and the position/pole sources.
    pub fn earth_de440(
        mu_earth: f64,
        central: impl CentralBodyState + 'static,
        pole: impl BodyPole + 'static,
    ) -> Self {
        Self::new(
            mu_earth,
            EARTH_J2_DE440,
            EARTH_EQUATORIAL_RADIUS_M_DE440,
            central,
            pole,
        )
    }

    /// The `J2` coefficient this term carries — exposed for reporting alongside
    /// the reference radius it is paired with.
    pub fn j2(&self) -> f64 {
        self.j2
    }

    /// The reference equatorial radius (m) `j2` is defined against.
    pub fn reference_radius(&self) -> f64 {
        self.r_eq
    }
}

impl ForceModel for Oblateness {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let planet = self.central.state_at(epoch)?;
        // Planet-centred position — the frame the J2 expansion is written in.
        let r = state.position - planet.position;
        let r_norm = r.norm();
        // Coincident with the planet's centre is degenerate, not a physical flyby;
        // fail loud rather than emit a non-finite acceleration (mirrors the
        // point-mass guard — one body here, so index 0).
        if r_norm == 0.0 || !r_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: r_norm,
            });
        }

        let axis = self.pole.pole_at(epoch)?;
        let axis_norm = axis.norm();
        // A zero-length or non-finite spin axis has no defined equator plane. Same
        // fail-loud treatment: a silently-normalised garbage axis would produce a
        // plausible-looking but wrong term.
        if axis_norm == 0.0 || !axis_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: axis_norm,
            });
        }
        let k_hat = axis / axis_norm;

        let r_hat = r / r_norm;
        // s = sin(latitude): the body's projection onto the spin axis.
        let s = r_hat.dot(&k_hat);

        // a = −(3/2)·J2·μ·R_eq²/r⁴ · [ (1 − 5s²)·r̂ + 2s·k̂ ]
        let scale = -1.5 * self.j2 * self.mu * self.r_eq * self.r_eq / r_norm.powi(4);
        Ok(scale * ((1.0 - 5.0 * s * s) * r_hat + 2.0 * s * k_hat))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::forces::relativity::FixedCentralBody;
    use crate::forces::CompositeForce;
    use crate::integrator::{Dop853, Integrator};

    /// Earth's `μ` (m³/s²) — the DE440-consistent value. Production pulls this
    /// through ANISE; the kernel-free tests use one literal so the closed-form
    /// oracle and the integration share it exactly.
    const MU_EARTH: f64 = 3.986_004_418e14;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    /// Earth's J2 term about the origin with the pole along `+z`.
    fn earth_j2_at_origin() -> Oblateness {
        Oblateness::earth_de440(
            MU_EARTH,
            FixedCentralBody::at_rest_origin(),
            FixedPole::frame_z(),
        )
    }

    /// The three signs that *are* the term, checked algebraically before any orbit
    /// is integrated (the cheapest possible catch for a flipped bracket).
    ///
    /// Over the **equator** J2 pulls **inward** (extra equatorial mass); over the
    /// **pole** it pushes **outward**, and by exactly twice the equatorial
    /// magnitude; at the magic latitude `sinφ = 1/√5` the bracket's `r̂` term
    /// vanishes and the acceleration is purely **anti-parallel to the spin axis**
    /// (due south in the northern hemisphere) — which is not the same as having no
    /// radial projection, since `k̂` is not perpendicular to `r̂` there.
    #[test]
    fn equatorial_inward_polar_outward_and_the_magic_latitude() {
        let term = earth_j2_at_origin();
        let r: f64 = 7.0e6;
        let base = 1.5 * EARTH_J2_DE440 * MU_EARTH * EARTH_EQUATORIAL_RADIUS_M_DE440.powi(2)
            / r.powi(4);

        // Equator: purely radial, inward, magnitude `base`.
        let eq = StateVector::from_components(r, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a_eq = term.acceleration(epoch0(), &eq).unwrap();
        assert!(
            a_eq.dot(&Vector3::x()) < 0.0,
            "J2 must pull inward over the equator: {a_eq:?}"
        );
        assert!(a_eq.y.abs() < 1e-30 && a_eq.z.abs() < 1e-30, "equatorial J2 is radial: {a_eq:?}");
        assert!(
            (a_eq.norm() - base).abs() < 1e-9 * base,
            "equatorial magnitude {} expected {base}",
            a_eq.norm()
        );

        // Pole: purely radial, outward, twice the equatorial magnitude.
        let pole = StateVector::from_components(0.0, 0.0, r, 0.0, 0.0, 0.0);
        let a_pole = term.acceleration(epoch0(), &pole).unwrap();
        assert!(
            a_pole.dot(&Vector3::z()) > 0.0,
            "J2 must push outward over the pole: {a_pole:?}"
        );
        assert!(
            (a_pole.norm() - 2.0 * base).abs() < 1e-9 * base,
            "polar magnitude {} expected {}",
            a_pole.norm(),
            2.0 * base
        );

        // Magic latitude sinφ = 1/√5: the bracket's r̂ term vanishes, leaving an
        // acceleration exactly anti-parallel to the spin axis. Pinned as
        // `a × k̂ = 0` (parallel) plus `a·k̂ < 0` (southward) — and the magnitude
        // `2s·base`, which a term that merely happened to be axial would miss.
        let s = 1.0 / 5f64.sqrt();
        let cos_phi = (1.0 - s * s).sqrt();
        let magic = StateVector::from_components(r * cos_phi, 0.0, r * s, 0.0, 0.0, 0.0);
        let a_magic = term.acceleration(epoch0(), &magic).unwrap();
        assert!(
            a_magic.cross(&Vector3::z()).norm() < 1e-9 * a_magic.norm(),
            "at the magic latitude J2 is purely axial: {a_magic:?}"
        );
        assert!(
            a_magic.dot(&Vector3::z()) < 0.0,
            "north of the equator the axial acceleration points south: {a_magic:?}"
        );
        assert!(
            (a_magic.norm() - 2.0 * s * base).abs() < 1e-9 * base,
            "magic-latitude magnitude {} expected {}",
            a_magic.norm(),
            2.0 * s * base
        );
    }

    /// `J2 · R_eq²` is the physical product, and the `1/r⁴` falloff is what makes
    /// the term heliocentrically irrelevant and close-flyby decisive. Doubling `r`
    /// must cut the magnitude by 16.
    #[test]
    fn magnitude_falls_off_as_inverse_fourth_power() {
        let term = earth_j2_at_origin();
        let near = StateVector::from_components(7.0e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        let far = StateVector::from_components(1.4e7, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a_near = term.acceleration(epoch0(), &near).unwrap().norm();
        let a_far = term.acceleration(epoch0(), &far).unwrap().norm();
        assert!(
            (a_near / a_far - 16.0).abs() < 1e-9,
            "1/r⁴ falloff: ratio {} expected 16",
            a_near / a_far
        );
    }

    /// The pole is genuinely a parameter: the *same* state under a spin axis
    /// rotated to `+x` yields the acceleration that state's new latitude implies.
    /// A term that quietly assumed `ẑ` would return the equatorial answer here.
    #[test]
    fn the_spin_axis_is_used_not_assumed_to_be_z() {
        let r = 7.0e6;
        let on_x = StateVector::from_components(r, 0.0, 0.0, 0.0, 0.0, 0.0);

        let z_pole = earth_j2_at_origin();
        let x_pole = Oblateness::earth_de440(
            MU_EARTH,
            FixedCentralBody::at_rest_origin(),
            FixedPole::new(Vector3::x()),
        );

        let a_z = z_pole.acceleration(epoch0(), &on_x).unwrap();
        let a_x = x_pole.acceleration(epoch0(), &on_x).unwrap();

        // Under a +z pole the point is on the equator (inward); under a +x pole it
        // is over the pole (outward, twice as big).
        assert!(a_z.x < 0.0 && a_x.x > 0.0, "pole choice must flip the sign: {a_z:?} vs {a_x:?}");
        assert!(
            (a_x.norm() / a_z.norm() - 2.0).abs() < 1e-9,
            "polar/equatorial magnitude ratio {} expected 2",
            a_x.norm() / a_z.norm()
        );
    }

    /// `J2 = 0` is exactly zero acceleration — the off-switch the shipping toggle
    /// relies on producing a bit-identical field.
    #[test]
    fn zero_j2_is_zero_acceleration() {
        let term = Oblateness::new(
            MU_EARTH,
            0.0,
            EARTH_EQUATORIAL_RADIUS_M_DE440,
            FixedCentralBody::at_rest_origin(),
            FixedPole::frame_z(),
        );
        let s = StateVector::from_components(7.0e6, 1.0e6, -2.0e6, 0.0, 0.0, 0.0);
        assert_eq!(term.acceleration(epoch0(), &s).unwrap(), Vector3::zeros());
    }

    /// A body at the planet's centre, and a zero-length spin axis, are both
    /// degenerate configurations — fail loud, never a `NaN` into the integrator.
    #[test]
    fn degenerate_configurations_fail_loud() {
        let term = earth_j2_at_origin();
        let at_centre = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(matches!(
            term.acceleration(epoch0(), &at_centre),
            Err(ForceError::Singularity { .. })
        ));

        let no_axis = Oblateness::earth_de440(
            MU_EARTH,
            FixedCentralBody::at_rest_origin(),
            FixedPole::new(Vector3::zeros()),
        );
        let s = StateVector::from_components(7.0e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(matches!(
            no_axis.acceleration(epoch0(), &s),
            Err(ForceError::Singularity { .. })
        ));
    }

    /// Right ascension of the ascending node from a state, via the angular
    /// momentum: `n̂ = ẑ × ĥ ∝ (−h_y, h_x, 0)`, so `Ω = atan2(h_x, −h_y)`.
    fn raan(state: &StateVector) -> f64 {
        let h = state.position.cross(&state.velocity);
        h.x.atan2(-h.y)
    }

    /// Integrate a circular orbit under point-mass gravity plus (optionally) the
    /// J2 term, sampling the RAAN once per orbital period — the stroboscopic
    /// sampling the 1PN test established, which measures the *secular* drift
    /// without the within-orbit oscillation contaminating it.
    ///
    /// Returns `(sample_times_s, raan_rad)`.
    fn integrate_and_sample_raan(
        j2: f64,
        r0: f64,
        inclination_rad: f64,
        orbits: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let v_circ = (MU_EARTH / r0).sqrt();
        // Node initially along +x: position on the node line, velocity carrying the
        // inclination. RAAN starts at 0 and the drift is read off from there.
        let start = StateVector::from_components(
            r0,
            0.0,
            0.0,
            0.0,
            v_circ * inclination_rad.cos(),
            v_circ * inclination_rad.sin(),
        );

        let mut model = CompositeForce::new().with(Box::new(PointMassGravity::new(vec![(
            MU_EARTH,
            FixedPerturber::at_origin(),
        )
            .into()])));
        model = model.with(Box::new(Oblateness::new(
            MU_EARTH,
            j2,
            EARTH_EQUATORIAL_RADIUS_M_DE440,
            FixedCentralBody::at_rest_origin(),
            FixedPole::frame_z(),
        )));

        let period = std::f64::consts::TAU * (r0 * r0 * r0 / MU_EARTH).sqrt();
        let stepper = Dop853::new().with_tolerances(1e-13, 1e-3);

        let mut state = start;
        let mut epoch = epoch0();
        let mut times = vec![0.0];
        let mut nodes = vec![raan(&state)];
        for k in 1..=orbits {
            state = stepper.step(&model, epoch, &state, period).unwrap();
            epoch = epoch.shifted_by_seconds(period);
            times.push(k as f64 * period);
            nodes.push(raan(&state));
        }
        (times, nodes)
    }

    /// Least-squares slope of `y` against `x` through the origin-free fit.
    fn slope(x: &[f64], y: &[f64]) -> f64 {
        let n = x.len() as f64;
        let mx = x.iter().sum::<f64>() / n;
        let my = y.iter().sum::<f64>() / n;
        let num: f64 = x.iter().zip(y).map(|(a, b)| (a - mx) * (b - my)).sum();
        let den: f64 = x.iter().map(|a| (a - mx).powi(2)).sum();
        num / den
    }

    /// Unwrap a RAAN series into a continuous angle so a least-squares slope is
    /// meaningful across the `±π` branch cut.
    fn unwrap(angles: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity(angles.len());
        let mut offset = 0.0;
        for (i, &a) in angles.iter().enumerate() {
            if i > 0 {
                let d = a + offset - out[i - 1];
                if d > std::f64::consts::PI {
                    offset -= std::f64::consts::TAU;
                } else if d < -std::f64::consts::PI {
                    offset += std::f64::consts::TAU;
                }
            }
            out.push(a + offset);
        }
        out
    }

    /// The headline isolation check (HANDOFF §6): the measured secular nodal drift
    /// reproduces the closed form
    /// `dΩ/dt = −(3/2)·n·J2·(R_eq/p)²·cos i`, computed with the **same** constants
    /// the integration uses (never a literal "−5°/day recalled for the ISS").
    ///
    /// A prograde orbit **regresses** (`dΩ/dt < 0`) — the sign that is easiest to
    /// get backwards and the one a flipped bracket would break.
    #[test]
    fn nodal_regression_matches_the_closed_form() {
        let r0 = 7.0e6;
        let inc = 51.6f64.to_radians();
        let orbits = 40;

        let (t, nodes) = integrate_and_sample_raan(EARTH_J2_DE440, r0, inc, orbits);
        let measured = slope(&t, &unwrap(&nodes));

        let n = (MU_EARTH / r0.powi(3)).sqrt();
        let expected = -1.5
            * n
            * EARTH_J2_DE440
            * (EARTH_EQUATORIAL_RADIUS_M_DE440 / r0).powi(2)
            * inc.cos();

        assert!(measured < 0.0, "a prograde orbit must regress, got {measured:.6e} rad/s");
        let rel = (measured - expected).abs() / expected.abs();
        assert!(
            rel < 0.02,
            "measured dΩ/dt {measured:.6e} vs closed form {expected:.6e} (rel {rel:.4})"
        );
    }

    /// The control that gives the measurement teeth: the identical integration with
    /// `J2 = 0` must show a drift orders of magnitude smaller — proving the
    /// regression above is the physics, not integrator drift in the node.
    #[test]
    fn without_j2_the_node_does_not_regress() {
        let r0 = 7.0e6;
        let inc = 51.6f64.to_radians();
        let orbits = 40;

        let (t, nodes) = integrate_and_sample_raan(EARTH_J2_DE440, r0, inc, orbits);
        let signal = slope(&t, &unwrap(&nodes)).abs();
        let (t0, nodes0) = integrate_and_sample_raan(0.0, r0, inc, orbits);
        let control = slope(&t0, &unwrap(&nodes0)).abs();

        assert!(
            control < 1e-3 * signal,
            "J2-off node drift {control:.3e} is not negligible against the signal {signal:.3e}"
        );
    }

    /// The inclination sign pair: a **retrograde** orbit (`i > 90°`, `cos i < 0`)
    /// makes the node **advance** instead of regress, and by the magnitude
    /// `|cos i|` implies. Catches a term that produced drift of the right size with
    /// no dependence on the orbit's orientation.
    #[test]
    fn retrograde_orbits_advance_the_node() {
        let r0 = 7.0e6;
        let orbits = 40;
        let inc = 120f64.to_radians();

        let (t, nodes) = integrate_and_sample_raan(EARTH_J2_DE440, r0, inc, orbits);
        let measured = slope(&t, &unwrap(&nodes));

        let n = (MU_EARTH / r0.powi(3)).sqrt();
        let expected = -1.5
            * n
            * EARTH_J2_DE440
            * (EARTH_EQUATORIAL_RADIUS_M_DE440 / r0).powi(2)
            * inc.cos();

        assert!(
            measured > 0.0 && expected > 0.0,
            "a retrograde orbit must advance the node, got {measured:.6e}"
        );
        let rel = (measured - expected).abs() / expected.abs();
        assert!(
            rel < 0.02,
            "retrograde dΩ/dt {measured:.6e} vs closed form {expected:.6e} (rel {rel:.4})"
        );
    }
}
