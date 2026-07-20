//! Yarkovsky thermal-recoil term — the decade-scale along-track dominator
//! (HANDOFF §5, §6, §7, §10).
//!
//! Sunlight absorbed and re-radiated by a spinning asteroid leaves with a thermal
//! lag, so the recoil has a component **along the orbital motion**. Over decades
//! this transverse push is the *dominant* driver of a real asteroid's along-track
//! drift — Bennu is the textbook case — and long-arc validation against Horizons
//! **will not match without it** (HANDOFF §272). It is listed as a known hard
//! problem precisely so its absence is not later mistaken for "my Rust is wrong."
//!
//! # The parametrization
//! Rather than a full thermophysical model (which needs spin axis, rotation
//! period, thermal inertia, size, density per body), this uses the **transverse
//! `A2` form** that JPL's Sentry / orbit-determination pipeline fits and
//! publishes (Farnocchia et al. 2013; Vokrouhlický et al.):
//!
//! ```text
//! a = A2 · (r₀ / r)^d · t̂,    t̂ = ĥ × r̂
//! ```
//!
//! a purely **transverse** acceleration (in the orbit plane, perpendicular to the
//! radius, along the prograde direction `ĥ × r̂`), scaling as `(r₀/r)^d` with a
//! reference distance `r₀ = 1 AU` and exponent `d ≈ 2`. `A2` (m/s²) carries the
//! sign of the drift: `A2 > 0` (prograde-rotating body) pushes along the motion
//! and drives the semi-major axis **outward** (`da/dt > 0`); `A2 < 0` (retrograde,
//! like Bennu) drives it inward. Its signature observable is the secular
//! semi-major-axis drift, orbit-averaged (time-weighted):
//!
//! ```text
//! ⟨da/dt⟩ = 2 · A2 · r₀² / (n · a² · (1 − e²))         (d = 2)
//! ```
//!
//! which the isolation test below reproduces (§6, "Yarkovsky alone produces the
//! right secular da/dt sign and magnitude").
//!
//! # Frame and the central-body state
//! Returns an acceleration in the **barycentric (SSB) ICRF** frame (HANDOFF §5),
//! but like 1PN the physics is heliocentric — the transverse direction is built
//! from the body's motion *relative to the Sun* — so it reuses the
//! [`super::relativity::CentralBodyState`] provider (position **and** velocity) to
//! form the relative `r` and `v`. [`FixedCentralBody`] keeps the isolation test
//! kernel-free.
//!
//! [`FixedCentralBody`]: super::relativity::FixedCentralBody

use super::relativity::CentralBodyState;
use super::relativity::FixedCentralBody;
use super::{ForceError, ForceModel};
use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Astronomical unit in metres — the default Yarkovsky reference distance `r₀`,
/// matching the value used across the crate and JPL's `A2` convention.
pub const AU_M: f64 = 1.495_978_707e11;

/// The Yarkovsky effect as a transverse `A2·(r₀/r)^d` acceleration (HANDOFF §5).
///
/// Holds the characteristic acceleration `A2` (m/s², signed), the reference
/// distance `r₀`, the falloff exponent `d`, and a [`CentralBodyState`] source for
/// the Sun's motion. As with the other terms the integrated body is a test
/// particle — its own mass is folded into the fitted `A2`, so this is an
/// acceleration.
pub struct YarkovskyA2 {
    /// Characteristic transverse acceleration at `r₀`, signed (m/s²).
    a2: f64,
    /// Reference heliocentric distance the scaling is normalised at (m).
    r0: f64,
    /// Falloff exponent (dimensionless); `2` for the standard `1/r²` insolation.
    d: f64,
    /// The central body (Sun) whose motion defines the heliocentric frame.
    central: Box<dyn CentralBodyState>,
}

impl YarkovskyA2 {
    /// Build the term with an explicit reference distance and exponent.
    pub fn new(a2: f64, r0: f64, d: f64, central: impl CentralBodyState + 'static) -> Self {
        Self {
            a2,
            r0,
            d,
            central: Box::new(central),
        }
    }

    /// The standard JPL parametrization: `r₀ = 1 AU`, `d = 2`.
    pub fn standard(a2: f64, central: impl CentralBodyState + 'static) -> Self {
        Self::new(a2, AU_M, 2.0, central)
    }

    /// Standard parametrization with the Sun pinned at the frame origin at rest —
    /// the kernel-free configuration the isolation tests use.
    pub fn sun_at_origin(a2: f64) -> Self {
        Self::standard(a2, FixedCentralBody::at_rest_origin())
    }
}

impl ForceModel for YarkovskyA2 {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let sun = self.central.state_at(epoch)?;
        let r = state.position - sun.position;
        let v = state.velocity - sun.velocity;

        let r_norm = r.norm();
        if r_norm == 0.0 || !r_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: r_norm,
            });
        }

        // Transverse (prograde, in-plane) unit vector t̂ = ĥ × r̂, where ĥ is the
        // orbit normal r×v. A degenerate radial state (r ∥ v → zero angular
        // momentum) has no defined transverse direction; fail loud rather than
        // divide by a zero norm.
        let h = r.cross(&v);
        let h_norm = h.norm();
        if h_norm == 0.0 || !h_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: h_norm,
            });
        }
        let r_hat = r / r_norm;
        let h_hat = h / h_norm;
        let t_hat = h_hat.cross(&r_hat);

        let magnitude = self.a2 * (self.r0 / r_norm).powf(self.d);
        Ok(magnitude * t_hat)
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

    /// At a **non-apsidal** state (r and v not perpendicular) the acceleration must
    /// be purely transverse — `a·r̂ = 0`, magnitude `A2·(r₀/r)^d`, and directed
    /// along `ĥ×r̂`, which off-apsis is **not** the velocity direction. The common
    /// wrong implementation picks `v̂`; this pins direction, magnitude, and units
    /// independently of any integration.
    #[test]
    fn acceleration_is_transverse_not_along_velocity() {
        let a2 = 1e-9;
        // r on +x; velocity has a radial (+x) component too → off-apsis, so v̂ is
        // tilted away from the pure-transverse ŷ.
        let rx = 0.7 * AU;
        let s = StateVector::from_components(rx, 0.0, 0.0, 5_000.0, 30_000.0, 0.0);
        let a = YarkovskyA2::sun_at_origin(a2).acceleration(epoch0(), &s).unwrap();

        // ĥ = r×v = (rx,0,0)×(vx,vy,0) = (0,0,rx·vy), vy>0 → +ẑ; t̂ = ẑ×x̂ = ŷ.
        let expected_mag = a2 * (AU / rx).powi(2);
        assert!(a.x.abs() < 1e-24, "must be perpendicular to r (a.x≈0): {a:?}");
        assert!(a.z.abs() < 1e-24, "planar motion stays planar: {a:?}");
        assert!(
            (a.y - expected_mag).abs() < 1e-6 * expected_mag,
            "a.y={} expected {expected_mag}",
            a.y
        );
        // Direction is ĥ×r̂ (=ŷ), NOT v̂: v has a large +x component, so v̂ is far
        // from ŷ. The cross-product-with-velocity mistake would tilt a into +x.
        let v = s.velocity;
        let cos_with_v = a.dot(&v) / (a.norm() * v.norm());
        assert!(
            cos_with_v < 0.99,
            "a must not be aligned with v̂ (cos={cos_with_v})"
        );
    }

    /// A body on the central mass, or in purely radial motion, has no transverse
    /// direction — fail loud.
    #[test]
    fn degenerate_states_fail_loud() {
        let term = YarkovskyA2::sun_at_origin(1e-9);
        // Coincident with the Sun.
        let on_sun = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(matches!(
            term.acceleration(epoch0(), &on_sun),
            Err(ForceError::Singularity { .. })
        ));
        // Purely radial motion: r ∥ v, zero angular momentum.
        let radial = StateVector::from_components(AU, 0.0, 0.0, 1_000.0, 0.0, 0.0);
        assert!(matches!(
            term.acceleration(epoch0(), &radial),
            Err(ForceError::Singularity { .. })
        ));
    }

    /// Osculating semi-major axis from the heliocentric state, via vis-viva:
    /// `a = 1/(2/r − v²/μ)`.
    fn osculating_a(state: &StateVector, mu: f64) -> f64 {
        let r = state.position.norm();
        let v2 = state.velocity.norm_squared();
        1.0 / (2.0 / r - v2 / mu)
    }

    /// Least-squares slope of `ys` against integer index `0..n`.
    fn slope_per_step(ys: &[f64]) -> f64 {
        let n = ys.len() as f64;
        let mean_x = (n - 1.0) / 2.0;
        let mean_y = ys.iter().sum::<f64>() / n;
        let (mut num, mut den) = (0.0, 0.0);
        for (i, &y) in ys.iter().enumerate() {
            let dx = i as f64 - mean_x;
            num += dx * (y - mean_y);
            den += dx * dx;
        }
        num / den
    }

    /// The **time-averaged** (uniform-in-mean-anomaly) secular da/dt for a
    /// transverse `A2·(r₀/r)^d` accel, computed straight from the Gauss planetary
    /// equation — independently of [`YarkovskyA2`] (the transverse magnitude comes
    /// from the `A2·(r₀/r)^d` scalar, never from the term's `acceleration()`), so
    /// a magnitude/direction bug in the term cannot cancel against the oracle.
    ///
    /// Sampling uniformly in mean anomaly `M` is uniform in *time* by construction
    /// — the weighting the advisor flagged as make-or-break: a uniform-in-true-
    /// anomaly average would be wrong by ~10% at e≈0.2.
    fn secular_da_dt_time_averaged(a2: f64, r0: f64, d: f64, a: f64, e: f64, mu: f64) -> f64 {
        let n = (mu / (a * a * a)).sqrt(); // mean motion
        let samples = 4000;
        let mut sum = 0.0;
        for i in 0..samples {
            let m = std::f64::consts::TAU * (i as f64) / (samples as f64);
            // Solve Kepler M = E − e·sinE for E (Newton; e is modest).
            let mut ecc = m;
            for _ in 0..60 {
                let f = ecc - e * ecc.sin() - m;
                let fp = 1.0 - e * ecc.cos();
                ecc -= f / fp;
            }
            let r = a * (1.0 - e * ecc.cos());
            // p/r = 1 + e·cosν, with p = a(1−e²).
            let one_plus_ecos_nu = a * (1.0 - e * e) / r;
            let a_t = a2 * (r0 / r).powf(d);
            // Gauss: da/dt = (2/(n√(1−e²)))·[e·sinν·a_R + (p/r)·a_T], a_R = 0.
            sum += 2.0 / (n * (1.0 - e * e).sqrt()) * one_plus_ecos_nu * a_t;
        }
        sum / (samples as f64)
    }

    /// Closed form of the same average (d = 2): `2·A2·r₀²/(n·a²·(1−e²))`. A cross-
    /// check on the numerical time-average's weighting algebra.
    fn secular_da_dt_closed_form(a2: f64, r0: f64, a: f64, e: f64, mu: f64) -> f64 {
        let n = (mu / (a * a * a)).sqrt();
        2.0 * a2 * r0 * r0 / (n * a * a * (1.0 - e * e))
    }

    /// Integrate an orbit under Newtonian gravity + the Yarkovsky term and measure
    /// the secular da/dt by **stroboscopic** sampling of the osculating semi-major
    /// axis (once per period, so the intra-orbit wiggle cancels), least-squares
    /// slope over `n_orbits`. `with_yarko = false` is the control run.
    fn measure_secular_da_dt(a2: f64, a: f64, e: f64, with_yarko: bool, n_orbits: usize) -> f64 {
        let r_peri = a * (1.0 - e);
        let v_peri = (MU_SUN * (2.0 / r_peri - 1.0 / a)).sqrt();
        let mut state = StateVector::from_components(r_peri, 0.0, 0.0, 0.0, v_peri, 0.0);
        let period = std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt();

        let mut model = CompositeForce::new().with(Box::new(PointMassGravity::new(vec![(
            MU_SUN,
            FixedPerturber::at_origin(),
        )
            .into()])));
        if with_yarko {
            model = model.with(Box::new(YarkovskyA2::sun_at_origin(a2)));
        }

        let stepper = Dop853::new().with_tolerances(1e-13, 1e-6);
        let mut samples = Vec::with_capacity(n_orbits + 1);
        let mut epoch = epoch0();
        samples.push(osculating_a(&state, MU_SUN));
        for _ in 0..n_orbits {
            state = stepper.step(&model, epoch, &state, period).unwrap();
            epoch = epoch.shifted_by_seconds(period);
            samples.push(osculating_a(&state, MU_SUN));
        }
        // slope is Δa per orbit; convert to per-second.
        slope_per_step(&samples) / period
    }

    #[test]
    fn oracle_time_average_matches_the_closed_form() {
        // The two independent oracles must agree — validates the uniform-M
        // weighting before either is trusted to judge the term.
        for &e in &[0.0, 0.2, 0.45] {
            let a = 1.1 * AU;
            let num = secular_da_dt_time_averaged(1e-9, AU, 2.0, a, e, MU_SUN);
            let cf = secular_da_dt_closed_form(1e-9, AU, a, e, MU_SUN);
            let rel = (num - cf).abs() / cf.abs();
            assert!(rel < 1e-4, "e={e}: numerical {num} vs closed form {cf} (rel {rel:.2e})");
        }
    }

    #[test]
    fn circular_orbit_drifts_at_the_transverse_rate() {
        // De-risk case: e = 0 removes the time-weighting ambiguity entirely, so
        // this alone validates the term's form, sign, and units.
        let a2 = 1e-9;
        let a = 1.0 * AU;
        let measured = measure_secular_da_dt(a2, a, 0.0, true, 40);
        let oracle = secular_da_dt_closed_form(a2, AU, a, 0.0, MU_SUN);
        assert!(measured > 0.0, "A2>0 must drift outward, got {measured} m/s");
        let rel = (measured - oracle).abs() / oracle;
        assert!(rel < 0.01, "measured {measured} m/s vs oracle {oracle} (rel {rel:.4})");
    }

    #[test]
    fn eccentric_orbit_matches_the_time_averaged_oracle() {
        // The eccentric case exercises the (r₀/r)^d scaling over a range of r and
        // the transverse-vs-velocity distinction; judged against the TIME-averaged
        // oracle (uniform-in-ν would be ~10% off here and fail).
        let a2 = 1e-9;
        let a = 1.0 * AU;
        let e = 0.2;
        let measured = measure_secular_da_dt(a2, a, e, true, 40);
        let oracle = secular_da_dt_time_averaged(a2, AU, 2.0, a, e, MU_SUN);
        assert!(measured > 0.0, "A2>0 must drift outward, got {measured} m/s");
        let rel = (measured - oracle).abs() / oracle;
        assert!(rel < 0.01, "measured {measured} m/s vs oracle {oracle} (rel {rel:.4})");
    }

    #[test]
    fn retrograde_a2_drifts_inward() {
        // Sign check: A2 < 0 (Bennu-like retrograde rotator) drives da/dt < 0.
        let measured = measure_secular_da_dt(-1e-9, 1.0 * AU, 0.15, true, 40);
        assert!(measured < 0.0, "A2<0 must drift inward, got {measured} m/s");
    }

    #[test]
    fn control_without_yarkovsky_shows_no_drift() {
        // The guard that gives the secular tests meaning: identical integration
        // with the term OFF must show a drift that is a small fraction of the
        // term-on signal — else a loose tolerance would "measure" integrator noise.
        let a2 = 1e-9;
        let a = 1.0 * AU;
        let e = 0.2;
        let control = measure_secular_da_dt(a2, a, e, false, 40);
        let signal = measure_secular_da_dt(a2, a, e, true, 40);
        assert!(
            control.abs() < 0.01 * signal.abs(),
            "control (Yarko off) drift {control} m/s must be ≪ signal {signal} m/s"
        );
    }
}
