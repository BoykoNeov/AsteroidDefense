//! Solar radiation pressure — the radial sibling of Yarkovsky (HANDOFF §5, §6).
//!
//! Sunlight carries momentum, so the photon flux the asteroid intercepts pushes it
//! **radially away from the Sun**. In JPL's non-gravitational model this is the
//! radial `A1` term, exactly parallel to the transverse `A2` Yarkovsky term this
//! module already carries — same `(r₀/r)^d` falloff, same test-particle framing,
//! only `+r̂` instead of `ĥ×r̂`. Because it is purely radial it produces **no
//! secular along-track drift** (that is Yarkovsky's job); a radial `1/r²` force
//! just rescales the central attraction, so its whole observable is a small change
//! in orbit *shape* — and, over a decade-long campaign, a small, honest b-plane
//! shift. Do not expect (or manufacture) a large one.
//!
//! # The cannonball parametrization
//! The standard flat-plate ("cannonball") model:
//!
//! ```text
//! a = a₁ · (r₀ / r)² · r̂,    a₁ = (Φ / c) · C_r · (A / m)
//! ```
//!
//! a radial acceleration scaling as `1/r²` with reference distance `r₀ = 1 AU`.
//! The characteristic acceleration `a₁` (m/s² at 1 AU) bundles the solar constant
//! `Φ` (total irradiance at 1 AU), the speed of light `c`, the radiation-pressure
//! coefficient `C_r` (1 for a perfect absorber, up to 2 for a perfect reflector),
//! and the area-to-mass ratio `A/m` (m²/kg). [`SolarRadiationPressure::from_physical`]
//! computes `a₁` from `C_r` and `A/m`; the term stores only the resulting `a₁`, so
//! the acceleration impl stays a single `1/r²` scale like Yarkovsky's.
//!
//! # The effective-μ signature (what the isolation test pins)
//! A radial outward `1/r²` force of magnitude `A_srp/r²` (with `A_srp = a₁·r₀²`)
//! subtracts directly from solar gravity's `μ_sun/r²`: the body orbits under an
//! **effective** gravitational parameter
//!
//! ```text
//! μ_eff = μ_sun · (1 − β),    β = A_srp / μ_sun.
//! ```
//!
//! Outward SRP makes `μ_eff` *smaller*, so a circular orbit's period gets *longer*
//! — the sign that is the load-bearing assertion and the easiest thing to get
//! backwards. The isolation tests below pin the sign (outward), the magnitude
//! (`a₁/r²`), and this period lengthening, at an exaggerated `β`; the shipping
//! config uses a physically plausible tiny `β` (~1e-9 for a sub-km rock) and
//! reports whatever b-plane shift it yields.
//!
//! # Frame and the central-body state
//! Returns a barycentric (SSB) ICRF acceleration (HANDOFF §5), but the physics is
//! heliocentric — the push is along the Sun→body direction — so it reuses the
//! [`super::relativity::CentralBodyState`] provider for the Sun. Unlike 1PN and
//! Yarkovsky it needs the Sun's **position only** (direction and distance), never
//! its velocity: the cannonball model omits the velocity-dependent
//! Poynting–Robertson drag (∝ v/c), which is negligible here. [`FixedCentralBody`]
//! keeps the isolation tests kernel-free.
//!
//! [`FixedCentralBody`]: super::relativity::FixedCentralBody

use super::relativity::{CentralBodyState, FixedCentralBody, SPEED_OF_LIGHT_M_S};
use super::{ForceError, ForceModel};
use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Astronomical unit in metres — the default SRP reference distance `r₀`, matching
/// the value used across the crate.
pub const AU_M: f64 = 1.495_978_707e11;

/// Total solar irradiance at 1 AU, W/m² — the "solar constant". The IAU/CODATA
/// nominal value; sets the momentum flux `Φ/c` a body intercepts at the reference
/// distance.
pub const SOLAR_CONSTANT_1AU_W_M2: f64 = 1361.0;

/// Solar radiation pressure at 1 AU on a perfectly absorbing surface, N/m² (Pa) —
/// `Φ / c`. Multiplied by `C_r · (A/m)` this gives the characteristic acceleration
/// `a₁` ([`SolarRadiationPressure::from_physical`]).
pub const RADIATION_PRESSURE_1AU_PA: f64 = SOLAR_CONSTANT_1AU_W_M2 / SPEED_OF_LIGHT_M_S;

/// Solar radiation pressure as a radial `a₁·(r₀/r)²` acceleration (HANDOFF §5).
///
/// Holds the characteristic acceleration `a₁` (m/s² at `r₀`, always outward), the
/// reference distance `r₀`, and a [`CentralBodyState`] source for the Sun's
/// position. Like the other non-gravitational terms the integrated body is a test
/// particle — its mass is folded into `a₁` via `A/m` — so this is an acceleration.
pub struct SolarRadiationPressure {
    /// Characteristic radial acceleration at `r₀`, m/s² (≥ 0; the outward sign is
    /// applied by the `+r̂` direction, so this magnitude is non-negative).
    a1: f64,
    /// Reference heliocentric distance the scaling is normalised at (m).
    r0: f64,
    /// The central body (Sun) whose position defines the outward direction.
    central: Box<dyn CentralBodyState>,
}

impl SolarRadiationPressure {
    /// Build the term from an explicit characteristic acceleration and reference
    /// distance. The `1/r²` falloff exponent is fixed (photon flux geometry).
    pub fn new(a1: f64, r0: f64, central: impl CentralBodyState + 'static) -> Self {
        Self {
            a1,
            r0,
            central: Box::new(central),
        }
    }

    /// Standard parametrization: `r₀ = 1 AU`, characteristic acceleration `a1`.
    pub fn standard(a1: f64, central: impl CentralBodyState + 'static) -> Self {
        Self::new(a1, AU_M, central)
    }

    /// Build from physical inputs: the radiation-pressure coefficient `C_r`
    /// (1 = absorber … 2 = reflector) and the area-to-mass ratio `A/m` (m²/kg).
    /// Computes `a₁ = (Φ/c)·C_r·(A/m)` at `r₀ = 1 AU` — the cannonball model.
    pub fn from_physical(cr: f64, area_to_mass_m2_per_kg: f64, central: impl CentralBodyState + 'static) -> Self {
        let a1 = RADIATION_PRESSURE_1AU_PA * cr * area_to_mass_m2_per_kg;
        Self::standard(a1, central)
    }

    /// Standard parametrization with the Sun pinned at the frame origin — the
    /// kernel-free configuration the isolation tests use.
    pub fn sun_at_origin(a1: f64) -> Self {
        Self::standard(a1, FixedCentralBody::at_rest_origin())
    }

    /// The characteristic acceleration `a₁` (m/s² at `r₀`) — exposed so a caller
    /// can compute the effective-μ factor `β = a₁·r₀²/μ_sun` for reporting.
    pub fn characteristic_acceleration(&self) -> f64 {
        self.a1
    }
}

impl ForceModel for SolarRadiationPressure {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let sun = self.central.state_at(epoch)?;
        // Sun→body direction: the push is radially outward along this.
        let r = state.position - sun.position;

        let r_norm = r.norm();
        if r_norm == 0.0 || !r_norm.is_finite() {
            // Coincident with the Sun has no defined outward direction — fail loud,
            // mirroring the point-mass singularity guard (one source, index 0).
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: r_norm,
            });
        }

        let r_hat = r / r_norm;
        let magnitude = self.a1 * (self.r0 / r_norm).powi(2);
        Ok(magnitude * r_hat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::forces::CompositeForce;
    use crate::integrator::{Dop853, Integrator};

    const MU_SUN: f64 = 1.327_124_400_18e20;
    const AU: f64 = 1.495_978_707e11;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    /// The cheap, integration-free pin: at an off-axis state the acceleration must
    /// be **radially outward** (`a ∥ +r̂`, `a·r̂ > 0`), with magnitude `a₁·(r₀/r)²`.
    /// This catches the single most likely bug — a flipped sign making SRP pull
    /// *toward* the Sun — before any orbit is integrated.
    #[test]
    fn acceleration_is_radial_and_outward() {
        let a1 = 1e-6;
        // An off-axis position so "outward" is a non-trivial direction.
        let rx = 0.7 * AU;
        let ry = 0.3 * AU;
        let s = StateVector::from_components(rx, ry, 0.0, 12_000.0, -5_000.0, 0.0);
        let a = SolarRadiationPressure::sun_at_origin(a1)
            .acceleration(epoch0(), &s)
            .unwrap();

        let r = s.position;
        let r_norm = r.norm();
        let r_hat = r / r_norm;
        let expected_mag = a1 * (AU / r_norm).powi(2);

        // Outward: positive projection on r̂, and the whole vector is r̂-parallel
        // (no transverse component — that would be the Yarkovsky mistake).
        assert!(a.dot(&r_hat) > 0.0, "SRP must push outward (a·r̂>0): {a:?}");
        let transverse = a - a.dot(&r_hat) * r_hat;
        assert!(transverse.norm() < 1e-18 * a.norm(), "SRP must be purely radial: {a:?}");
        assert!(
            (a.norm() - expected_mag).abs() < 1e-9 * expected_mag,
            "magnitude {} expected {expected_mag}",
            a.norm()
        );
    }

    /// The `(r₀/r)²` falloff: doubling the heliocentric distance quarters the
    /// magnitude. Pins the exponent independently of the effective-μ integration.
    #[test]
    fn magnitude_falls_off_as_inverse_square() {
        let a1 = 1e-6;
        let term = SolarRadiationPressure::sun_at_origin(a1);
        let near = StateVector::from_components(AU, 0.0, 0.0, 0.0, 0.0, 0.0);
        let far = StateVector::from_components(2.0 * AU, 0.0, 0.0, 0.0, 0.0, 0.0);
        let a_near = term.acceleration(epoch0(), &near).unwrap().norm();
        let a_far = term.acceleration(epoch0(), &far).unwrap().norm();
        assert!((a_near - a1).abs() < 1e-12 * a1, "at r₀ the magnitude is a₁");
        assert!((a_far - a1 / 4.0).abs() < 1e-12 * a1, "at 2 r₀ the magnitude is a₁/4");
    }

    /// `from_physical` reproduces the cannonball formula `a₁ = (Φ/c)·C_r·(A/m)`.
    /// A sub-km rock's β lands at the ~1e-9 the module note quotes — the sanity
    /// that the shipping value is tiny, not amplified.
    #[test]
    fn physical_constructor_matches_the_cannonball_formula() {
        // A 300 m diameter rock, density 2000 kg/m³: A/m = 3/(4·r·ρ) ≈ 2.5e-6 m²/kg.
        let radius = 150.0;
        let density = 2000.0;
        let area_to_mass = 3.0 / (4.0 * radius * density);
        let cr = 1.3;
        let term = SolarRadiationPressure::from_physical(cr, area_to_mass, FixedCentralBody::at_rest_origin());
        let expected_a1 = RADIATION_PRESSURE_1AU_PA * cr * area_to_mass;
        assert!(
            (term.characteristic_acceleration() - expected_a1).abs() < 1e-18,
            "a₁ = {}, expected {expected_a1}",
            term.characteristic_acceleration()
        );
        // β = a₁·AU²/μ_sun should be ~1e-9 — physically tiny, as the note promises.
        let beta = expected_a1 * AU * AU / MU_SUN;
        assert!(beta > 1e-10 && beta < 1e-8, "sub-km β should be ~1e-9, got {beta}");
    }

    /// A body coincident with the Sun has no defined outward direction — fail loud.
    /// (Unlike Yarkovsky, purely radial *motion* is fine for SRP: it never forms a
    /// cross product, so only a zero separation is degenerate.)
    #[test]
    fn coincident_with_sun_fails_loud() {
        let term = SolarRadiationPressure::sun_at_origin(1e-6);
        let on_sun = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(matches!(
            term.acceleration(epoch0(), &on_sun),
            Err(ForceError::Singularity { .. })
        ));
    }

    /// Radius of the body's position at each of `samples` points along one full
    /// integration of `total_seconds`, plus the closing gap `|r(T) − r(0)|`. Uses
    /// the **geometric** position directly (never osculating elements, which would
    /// report a spurious a≠r on a μ_eff orbit evaluated with μ_sun).
    fn integrate_circular(a1: f64, r0: f64, with_srp: bool, total_seconds: f64, samples: usize) -> (Vec<f64>, f64) {
        // Effective-μ the circular condition is built for: μ_eff = μ_sun − A_srp,
        // A_srp = a1·r0². With SRP off the initial speed is sub-circular for μ_sun.
        let a_srp = a1 * r0 * r0;
        let mu_eff = MU_SUN - a_srp;
        let v_circ = (mu_eff / r0).sqrt();
        let start = StateVector::from_components(r0, 0.0, 0.0, 0.0, v_circ, 0.0);

        let mut model = CompositeForce::new().with(Box::new(PointMassGravity::new(vec![(
            MU_SUN,
            FixedPerturber::at_origin(),
        )
            .into()])));
        if with_srp {
            model = model.with(Box::new(SolarRadiationPressure::sun_at_origin(a1)));
        }

        let stepper = Dop853::new().with_tolerances(1e-13, 1e-6);
        let dt = total_seconds / samples as f64;
        let mut state = start;
        let mut epoch = epoch0();
        let mut radii = Vec::with_capacity(samples + 1);
        radii.push(state.position.norm());
        for _ in 0..samples {
            state = stepper.step(&model, epoch, &state, dt).unwrap();
            epoch = epoch.shifted_by_seconds(dt);
            radii.push(state.position.norm());
        }
        let closing_gap = (state.position - start.position).norm();
        (radii, closing_gap)
    }

    /// The headline isolation test: with SRP on, a body launched at the μ_eff
    /// circular speed traces an **exact circle** (radius constant) and closes after
    /// the μ_eff period `T_eff = 2π√(r₀³/μ_eff)`. Because μ_eff < μ_sun, `T_eff` is
    /// **longer** than the Newtonian period — the outward-SRP sign, measured.
    #[test]
    fn srp_makes_a_circular_orbit_under_effective_mu() {
        // Exaggerated β for a clean signal (shipping β is ~1e-9, integrator noise).
        let beta = 0.02;
        let r0 = 1.0 * AU;
        let a_srp = beta * MU_SUN;
        let a1 = a_srp / (r0 * r0);
        let mu_eff = MU_SUN - a_srp;

        let t_eff = std::f64::consts::TAU * (r0 * r0 * r0 / mu_eff).sqrt();
        let t_newton = std::f64::consts::TAU * (r0 * r0 * r0 / MU_SUN).sqrt();
        assert!(t_eff > t_newton, "outward SRP must lengthen the period: {t_eff} vs {t_newton}");

        let (radii, closing_gap) = integrate_circular(a1, r0, true, t_eff, 400);
        // Radius stays r0 — the orbit is a genuine circle under μ_eff.
        for r in &radii {
            assert!((r - r0).abs() < 1e-6 * r0, "radius wandered off r₀: {r} vs {r0}");
        }
        // Closes to itself after one μ_eff period.
        assert!(closing_gap < 1e-6 * r0, "orbit did not close after T_eff: gap {closing_gap} m");
    }

    /// The control that gives the effective-μ test teeth: the **same** initial
    /// state (μ_eff circular speed) with SRP **off** is sub-circular for μ_sun, so
    /// it must NOT trace a circle and must NOT close at `T_eff` — proving SRP, not
    /// the integrator, produced the circular μ_eff orbit above.
    #[test]
    fn without_srp_the_same_state_is_not_circular() {
        let beta = 0.02;
        let r0 = 1.0 * AU;
        let a_srp = beta * MU_SUN;
        let a1 = a_srp / (r0 * r0);
        let mu_eff = MU_SUN - a_srp;
        let t_eff = std::f64::consts::TAU * (r0 * r0 * r0 / mu_eff).sqrt();

        let (radii, closing_gap) = integrate_circular(a1, r0, false, t_eff, 400);
        // Sub-circular speed for μ_sun → the body falls inward; radius must dip
        // well below r0 somewhere on the arc.
        let min_r = radii.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(min_r < 0.99 * r0, "SRP-off orbit should fall inward, min radius {min_r} vs {r0}");
        // And it must not close at the μ_eff period.
        assert!(closing_gap > 1e-3 * r0, "SRP-off orbit closed at T_eff — SRP was not load-bearing");
    }
}
