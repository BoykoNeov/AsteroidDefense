//! `OrbitalElements` — classical Keplerian elements and the element↔state map.
//!
//! Classical (Keplerian) elements `{a, e, i, Ω, ω, ν}` for a **bound elliptical**
//! orbit (`0 ≤ e < 1`), and the pure-geometry conversions to and from a
//! Cartesian [`StateVector`] about an attractor of gravitational parameter `μ`
//! (SI, m³/s²). This is a *static* map at one epoch — there is no Kepler-equation
//! solve here; time evolution is the propagator's job (HANDOFF §10.4).
//!
//! # Scope
//! - **Elliptical only.** Parabolic/hyperbolic states (`e ≥ 1`) are out of scope
//!   for task-3; the Earth-relative *encounter hyperbola* is handled in the
//!   b-plane work (§10.8), a different frame. [`OrbitalElements::from_state`]
//!   reports [`ElementsError::NonElliptical`] rather than silently returning
//!   nonsense.
//! - Uses the true anomaly `ν` directly — no mean/eccentric anomaly.
//!
//! # The singularity conventions (HANDOFF §10.3)
//! Two classical elements are *gauge* — undefined at coordinate singularities:
//! `ω` is undefined for a circular orbit (`e → 0`, no periapsis), and `Ω` is
//! undefined for an equatorial orbit (`i → 0` or `i → π`, no ascending node).
//! [`from_state`](OrbitalElements::from_state) picks a definite convention there
//! so a valid element set always exists and no `0/0` ever reaches an `acos`:
//! - circular & inclined  → `ω = 0`, `ν` = argument of latitude (node→body);
//! - elliptical & equatorial → `Ω = 0`, `ω` = longitude of periapsis (x̂→periapsis);
//! - circular & equatorial → `Ω = 0`, `ω = 0`, `ν` = true longitude (x̂→body).
//!
//! Because the round-trip is validated on the **state** (the physical invariant),
//! not on the gauge angles, these choices only need to be self-consistent with
//! [`to_state`](OrbitalElements::to_state) — which they are, since the mislabelled
//! angle and the anomaly always sum to the same physical in-plane angle.

use crate::state::StateVector;
use nalgebra::{Matrix3, Vector3};
use std::f64::consts::TAU;

/// Threshold below which eccentricity is treated as circular / inclination as
/// equatorial (via `sin i`). Small enough that every physically-meaningful orbit
/// takes the exact standard branch; large enough to catch true singularities and
/// denormal noise before a gauge angle divides by zero.
const SINGULARITY_TOL: f64 = 1e-11;

/// Failure modes of [`OrbitalElements::from_state`].
#[derive(Debug, Clone, PartialEq)]
pub enum ElementsError {
    /// The state is unbound or parabolic (`e ≥ 1`, i.e. non-negative specific
    /// orbital energy). Out of scope for the Keplerian element set (§10.3).
    NonElliptical { eccentricity: f64 },
    /// The state is degenerate (zero radius or zero angular momentum) and has no
    /// well-defined orbit.
    Degenerate,
}

impl std::fmt::Display for ElementsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ElementsError::NonElliptical { eccentricity } => {
                write!(f, "state is non-elliptical (e = {eccentricity:.6} ≥ 1)")
            }
            ElementsError::Degenerate => {
                write!(f, "degenerate state (zero radius or angular momentum)")
            }
        }
    }
}

impl std::error::Error for ElementsError {}

/// Classical Keplerian elements for a bound elliptical orbit.
///
/// Angles in radians. `inclination ∈ [0, π]`; `raan`, `arg_periapsis`,
/// `true_anomaly` are wrapped to `[0, 2π)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitalElements {
    /// Semi-major axis `a`, metres. Positive for an ellipse.
    pub semi_major_axis: f64,
    /// Eccentricity `e`, `0 ≤ e < 1`.
    pub eccentricity: f64,
    /// Inclination `i`, radians, `[0, π]`.
    pub inclination: f64,
    /// Right ascension of the ascending node `Ω`, radians, `[0, 2π)`.
    pub raan: f64,
    /// Argument of periapsis `ω`, radians, `[0, 2π)`.
    pub arg_periapsis: f64,
    /// True anomaly `ν`, radians, `[0, 2π)`.
    pub true_anomaly: f64,
}

impl OrbitalElements {
    /// Build the element set. `raan`, `arg_periapsis`, and `true_anomaly` are
    /// periodic, so they are losslessly *wrapped* to `[0, 2π)`. `inclination` is
    /// **not** periodic — an out-of-range `i` denotes a genuinely different orbit,
    /// not an equivalent one, so a lossy clamp would silently corrupt it. In debug
    /// builds an `i ∉ [0, π]` trips a `debug_assert` (the crate's fail-loud style);
    /// release builds still clamp as a last-ditch guard rather than feed a bogus
    /// `i` into the trig. Internal callers ([`OrbitalElements::from_state`],
    /// [`crate::propagator::KeplerPropagator::elements_at`]) always supply
    /// `i ∈ [0, π]`, so the assert only fires on out-of-contract external input.
    pub fn new(
        semi_major_axis: f64,
        eccentricity: f64,
        inclination: f64,
        raan: f64,
        arg_periapsis: f64,
        true_anomaly: f64,
    ) -> Self {
        debug_assert!(
            (0.0..=std::f64::consts::PI).contains(&inclination),
            "inclination {inclination} out of range [0, π]: silently clamping it would build a different orbit"
        );
        Self {
            semi_major_axis,
            eccentricity,
            inclination: inclination.clamp(0.0, std::f64::consts::PI),
            raan: wrap_2pi(raan),
            arg_periapsis: wrap_2pi(arg_periapsis),
            true_anomaly: wrap_2pi(true_anomaly),
        }
    }

    /// Semi-latus rectum `p = a(1 - e²)`, metres.
    pub fn semi_latus_rectum(&self) -> f64 {
        self.semi_major_axis * (1.0 - self.eccentricity * self.eccentricity)
    }

    /// Convert to a Cartesian [`StateVector`] about an attractor of gravitational
    /// parameter `mu` (SI, m³/s²). Pure geometry — the perifocal state at `ν`
    /// rotated into the frame by `Rz(Ω)·Rx(i)·Rz(ω)`.
    pub fn to_state(&self, mu: f64) -> StateVector {
        let e = self.eccentricity;
        let nu = self.true_anomaly;
        let p = self.semi_latus_rectum();

        let (sin_nu, cos_nu) = nu.sin_cos();
        let r = p / (1.0 + e * cos_nu);

        // Perifocal (PQW) frame: P̂ toward periapsis, Q̂ 90° ahead in-plane.
        let r_pqw = Vector3::new(r * cos_nu, r * sin_nu, 0.0);
        let sqrt_mu_over_p = (mu / p).sqrt();
        let v_pqw = Vector3::new(-sin_nu, e + cos_nu, 0.0) * sqrt_mu_over_p;

        let rot = perifocal_to_inertial(self.inclination, self.raan, self.arg_periapsis);
        StateVector::new(rot * r_pqw, rot * v_pqw)
    }

    /// Recover Keplerian elements from a Cartesian [`StateVector`] about an
    /// attractor of gravitational parameter `mu` (SI, m³/s²).
    ///
    /// Applies the singularity conventions documented on the module: a valid
    /// element set is always returned for a bound, non-degenerate state, even at
    /// `e = 0` and/or `i ∈ {0, π}`. Returns [`ElementsError`] for a degenerate
    /// (zero radius / angular momentum) or non-elliptical (`e ≥ 1`) state.
    pub fn from_state(state: StateVector, mu: f64) -> Result<Self, ElementsError> {
        let r_vec = state.position;
        let v_vec = state.velocity;
        let r = r_vec.norm();

        let h_vec = r_vec.cross(&v_vec);
        let h = h_vec.norm();
        if r == 0.0 || h == 0.0 {
            return Err(ElementsError::Degenerate);
        }

        let v2 = v_vec.dot(&v_vec);
        let rv = r_vec.dot(&v_vec);

        // Eccentricity vector (points toward periapsis), magnitude = e.
        let e_vec = (r_vec * (v2 - mu / r) - v_vec * rv) / mu;
        let ecc = e_vec.norm();
        if ecc >= 1.0 {
            return Err(ElementsError::NonElliptical { eccentricity: ecc });
        }

        // Semi-major axis from the vis-viva / specific-energy relation.
        let energy = 0.5 * v2 - mu / r;
        // energy < 0 for a bound orbit, guaranteed here since e < 1.
        let a = -mu / (2.0 * energy);

        // Inclination via atan2(n, h_z) keeps i in [0, π] with no acos clamp.
        let n_vec = Vector3::new(-h_vec.y, h_vec.x, 0.0); // ẑ × h
        let n = n_vec.norm();
        let inclination = n.atan2(h_vec.z);

        // sin i as a dimensionless equatorial test (n = h·sin i).
        let equatorial = (n / h) < SINGULARITY_TOL;
        let circular = ecc < SINGULARITY_TOL;

        let (raan, arg_periapsis, true_anomaly) = match (circular, equatorial) {
            // General case: node and periapsis both well-defined.
            (false, false) => {
                let raan = wrap_2pi(n_vec.y.atan2(n_vec.x));
                let mut argp = acos_clamped(n_vec.dot(&e_vec) / (n * ecc));
                if e_vec.z < 0.0 {
                    argp = TAU - argp;
                }
                let nu = true_anomaly_from(&e_vec, &r_vec, ecc, r, rv);
                (raan, argp, nu)
            }
            // Circular & inclined: no periapsis. ω = 0, ν = argument of latitude
            // (angle from the ascending node to the body, in-plane).
            (true, false) => {
                let raan = wrap_2pi(n_vec.y.atan2(n_vec.x));
                let mut u = acos_clamped(n_vec.dot(&r_vec) / (n * r));
                if r_vec.z < 0.0 {
                    u = TAU - u;
                }
                (raan, 0.0, u)
            }
            // Elliptical & equatorial: no node. Ω = 0, ω = longitude of periapsis
            // (angle from x̂ to the eccentricity vector, in-plane). For a
            // retrograde orbit (hz < 0) the Rx(π) tilt reverses the in-plane
            // sense, so the frame angle is negated to stay consistent with
            // `to_state`.
            (false, true) => {
                let lon_peri = wrap_2pi(e_vec.y.atan2(e_vec.x));
                let argp = if h_vec.z < 0.0 {
                    wrap_2pi(-lon_peri)
                } else {
                    lon_peri
                };
                let nu = true_anomaly_from(&e_vec, &r_vec, ecc, r, rv);
                (0.0, argp, nu)
            }
            // Circular & equatorial: neither node nor periapsis. Ω = ω = 0,
            // ν = true longitude (angle from x̂ to the body), sense-reversed for
            // a retrograde orbit as above.
            (true, true) => {
                let lon = wrap_2pi(r_vec.y.atan2(r_vec.x));
                let nu = if h_vec.z < 0.0 { wrap_2pi(-lon) } else { lon };
                (0.0, 0.0, nu)
            }
        };

        Ok(OrbitalElements {
            semi_major_axis: a,
            eccentricity: ecc,
            inclination,
            raan,
            arg_periapsis,
            true_anomaly,
        })
    }
}

/// True anomaly from the eccentricity and position vectors: the angle from
/// periapsis (e_vec) to the body (r_vec), disambiguated by the sign of `r·v`
/// (positive = outbound, `ν ∈ [0, π)`).
fn true_anomaly_from(e_vec: &Vector3<f64>, r_vec: &Vector3<f64>, ecc: f64, r: f64, rv: f64) -> f64 {
    let mut nu = acos_clamped(e_vec.dot(r_vec) / (ecc * r));
    if rv < 0.0 {
        nu = TAU - nu;
    }
    nu
}

/// Perifocal→inertial rotation `Rz(Ω)·Rx(i)·Rz(ω)` (the classical 3-1-3 sequence).
fn perifocal_to_inertial(inclination: f64, raan: f64, arg_periapsis: f64) -> Matrix3<f64> {
    let (si, ci) = inclination.sin_cos();
    let (so, co) = raan.sin_cos();
    let (sw, cw) = arg_periapsis.sin_cos();

    Matrix3::new(
        co * cw - so * sw * ci,
        -co * sw - so * cw * ci,
        so * si,
        so * cw + co * sw * ci,
        -so * sw + co * cw * ci,
        -co * si,
        sw * si,
        cw * si,
        ci,
    )
}

/// `acos` with the argument clamped to `[-1, 1]` so floating-point overshoot at
/// the singularities can never produce a `NaN`.
fn acos_clamped(x: f64) -> f64 {
    x.clamp(-1.0, 1.0).acos()
}

/// Wrap an angle to `[0, 2π)`.
fn wrap_2pi(angle: f64) -> f64 {
    let m = angle % TAU;
    if m < 0.0 {
        m + TAU
    } else {
        m
    }
}
