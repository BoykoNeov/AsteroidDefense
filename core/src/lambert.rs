//! `lambert` — the two-point boundary-value transfer solver (HANDOFF §8).
//!
//! Lambert's problem: given two position vectors `r1`, `r2` about a single
//! attractor of gravitational parameter `μ`, and a time of flight `Δt` between
//! them, find the *conic* that connects them — i.e. the departure velocity `v1`
//! at `r1` and the arrival velocity `v2` at `r2`. This is the primitive the
//! **mission-design / porkchop** layer (`mission.rs`) is built on: it turns "a Δv
//! appears at the asteroid" into "a spacecraft launches from Earth on date A,
//! coasts, and arrives at the asteroid on date B" — the deliverability the MVP
//! deflection curve assumes rather than proves (§7, §180).
//!
//! # This is a two-body solver, and that is correct for the *planning* layer
//! The transfer arc here is pure two-body (Sun-only) — which is exactly what a
//! real interplanetary cruise *is*. It is **not** a display-grade shortcut of the
//! sort deleted from the frontend (the honest-hit/miss physics stays in the full
//! `n`-body field). The honesty conditions the mission layer must uphold: the
//! *endpoints* are real (`r1` = Earth from the ephemeris, `r2` = the asteroid from
//! its integrated trajectory), Lambert only *sizes* the delivery and never
//! replaces the propagation, and its outputs are labelled patched-conic planning
//! estimates. This module owns only the conic solve; those framing conditions are
//! the caller's contract.
//!
//! # Algorithm — universal variables (Bate/Mueller/White; Curtis Algorithm 5.2)
//! A single formulation covers elliptic, parabolic, and hyperbolic transfers via
//! the Stumpff functions `C(z)`, `S(z)` and a Newton iteration on the universal
//! anomaly variable `z`. First cut: **single revolution, "short-way" prograde**
//! (`Δν < π` when the transfer angular momentum points along `+z`). Multi-rev and
//! the retrograde/long-way branch are a later upgrade; the [`prograde`] flag
//! selects direction and the choice is surfaced to the caller, per the
//! no-silent-defaults rule.
//!
//! # The 180° singularity is a real gap, not an error to hide
//! When `r1` and `r2` are collinear (`Δν → 0` or `π`) the transfer plane is
//! undefined and the solve is singular. This returns
//! [`LambertError::DegenerateGeometry`] — which the porkchop grid renders as an
//! empty cell, *not* a `NaN` that would poison the whole heatmap (the same
//! discipline the b-plane 180° case already follows).
//!
//! # Kernel-free by construction
//! Pure geometry over caller-supplied vectors — no ephemeris, no `μ` of its own
//! (the caller passes the *same* `μ_sun` the point-mass Sun term uses; a second
//! hardcoded constant would be the silent bias that bit the 1PN term). Validated
//! in isolation against the analytic [`KeplerPropagator`](crate::KeplerPropagator)
//! round-trip, an independent published worked example, and the free
//! energy/angular-momentum invariants of the transfer conic.

use nalgebra::Vector3;

/// The velocities that close a Lambert transfer: the conic through `r1` and `r2`
/// with the requested time of flight, evaluated at each endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LambertSolution {
    /// Heliocentric velocity at the departure point `r1`, m/s. The launch layer
    /// differences this against Earth's velocity to get the hyperbolic excess
    /// (and `C3`).
    pub v1: Vector3<f64>,
    /// Heliocentric velocity at the arrival point `r2`, m/s. The impact layer
    /// differences this against the asteroid's velocity to get the arrival
    /// relative velocity that aims and sizes the kinetic impactor.
    pub v2: Vector3<f64>,
}

/// Why a Lambert solve did not produce a transfer.
///
/// A single concrete enum, matching the crate's object-safe error style. The
/// mission layer maps every variant to a "no transfer here" porkchop gap; kept
/// distinct so the isolation tests can assert *which* failure a geometry hits.
#[derive(Debug, Clone, PartialEq)]
pub enum LambertError {
    /// `r1` and `r2` are collinear (`Δν ≈ 0` or `π`): the transfer plane is
    /// undefined and the solve is singular. Rendered as a porkchop gap, never a
    /// `NaN`.
    DegenerateGeometry {
        /// The transfer angle `Δν` (rad) that triggered the guard.
        transfer_angle_rad: f64,
    },
    /// The Newton iteration on the universal variable did not reach the
    /// time-of-flight tolerance within the iteration cap. Surfaced rather than
    /// returning a bad root; for single-rev transfers this indicates a geometry
    /// outside the short-way branch (e.g. one needing multiple revolutions).
    NonConvergence {
        /// Iterations spent before giving up.
        iterations: u32,
        /// Final time-of-flight residual, seconds.
        residual_seconds: f64,
    },
    /// A degenerate input: non-positive time of flight or `μ`, or a zero-length
    /// position vector.
    InvalidInput {
        /// The offending time of flight, seconds.
        tof_seconds: f64,
        /// The offending gravitational parameter, m³/s².
        mu: f64,
    },
}

impl std::fmt::Display for LambertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LambertError::DegenerateGeometry { transfer_angle_rad } => write!(
                f,
                "degenerate Lambert geometry: r1, r2 collinear (Δν = {transfer_angle_rad:.6} rad); transfer plane undefined"
            ),
            LambertError::NonConvergence {
                iterations,
                residual_seconds,
            } => write!(
                f,
                "Lambert iteration did not converge after {iterations} steps (residual {residual_seconds:.3e} s); outside the single-rev short-way branch"
            ),
            LambertError::InvalidInput { tof_seconds, mu } => write!(
                f,
                "invalid Lambert input (Δt = {tof_seconds:.6e} s, μ = {mu:.6e}); need Δt > 0, μ > 0, |r| > 0"
            ),
        }
    }
}

impl std::error::Error for LambertError {}

/// Newton iteration cap on the universal variable. Single-rev short-way
/// transfers converge quadratically in well under this; the cap is a backstop
/// that turns a non-converging geometry into a clean [`LambertError::NonConvergence`]
/// (a porkchop gap) rather than a hang.
const MAX_ITERS: u32 = 100;

/// Time-of-flight convergence tolerance, *relative* to the requested `Δt`. At
/// `1e-11` the transfer arrives at `r2` to a fraction of a second over a
/// multi-year cruise — far tighter than any planning use needs, and cheap.
const TOF_REL_TOL: f64 = 1e-11;

/// Below this `|sin Δν|` the endpoints are treated as collinear and the geometry
/// is [`LambertError::DegenerateGeometry`]. `sin Δν` vanishes at both `Δν = 0`
/// (parallel, radial transfer) and `Δν = π` (the 180° singularity), and the
/// coefficient `A` that carries the plane is `∝ sin Δν` near `π`.
const SIN_DNU_EPS: f64 = 1e-9;

/// Solve Lambert's problem: the single-revolution conic through `r1` and `r2`
/// with time of flight `tof_seconds` about an attractor of gravitational
/// parameter `mu` (SI: metres, seconds, m³/s²).
///
/// `prograde = true` selects the transfer whose angular momentum points along
/// the **`+z` axis of the frame `r1`/`r2` are given in** (the short way for
/// `Δν < π`). For heliocentric **ICRF** inputs that is celestial north, and
/// Earth and near-ecliptic targets orbit prograde about it — so `true` is the
/// right default here. (The distinction from the ecliptic pole is a ~23.4°
/// tilt; it does not matter for the sign of `+z`·`ĥ` on near-ecliptic transfers,
/// but the reference is ICRF, not the ecliptic.) `prograde = false` gives the
/// retrograde / long-way branch. This first cut is single-revolution only.
///
/// Returns the departure/arrival velocities, or a [`LambertError`] for a
/// degenerate geometry (collinear endpoints), a non-converging solve (a geometry
/// outside the short-way single-rev branch), or invalid input.
pub fn lambert_universal(
    r1: Vector3<f64>,
    r2: Vector3<f64>,
    tof_seconds: f64,
    mu: f64,
    prograde: bool,
) -> Result<LambertSolution, LambertError> {
    // Fail closed on non-finite / non-positive inputs (NaN fails every `>`).
    let r1n = r1.norm();
    let r2n = r2.norm();
    let inputs_ok = tof_seconds.is_finite()
        && tof_seconds > 0.0
        && mu.is_finite()
        && mu > 0.0
        && r1n > 0.0
        && r2n > 0.0;
    if !inputs_ok {
        return Err(LambertError::InvalidInput { tof_seconds, mu });
    }

    // Transfer angle Δν, resolved into [0, 2π) by the requested direction. The
    // z-component of r1×r2 tells whether the short arc is prograde: for prograde
    // motion a negative z means the short arc runs the "wrong" way, so take the
    // long arc (2π − Δν); retrograde is the mirror.
    let cos_dnu = (r1.dot(&r2) / (r1n * r2n)).clamp(-1.0, 1.0);
    let cross_z = r1.x * r2.y - r1.y * r2.x;
    let mut dnu = cos_dnu.acos(); // principal value in [0, π]
    let take_long_way = if prograde {
        cross_z < 0.0
    } else {
        cross_z >= 0.0
    };
    if take_long_way {
        dnu = std::f64::consts::TAU - dnu;
    }

    // A carries the transfer plane; A ∝ sin Δν vanishes at the 180° singularity
    // (and Δν = 0 is an undefined radial transfer). Guard both as degenerate.
    let sin_dnu = dnu.sin();
    if sin_dnu.abs() < SIN_DNU_EPS {
        return Err(LambertError::DegenerateGeometry {
            transfer_angle_rad: dnu,
        });
    }
    let a_coef = sin_dnu * (r1n * r2n / (1.0 - dnu.cos())).sqrt();

    // Newton on the universal variable z (= χ², the ratio governing the conic
    // type: z > 0 elliptic, z = 0 parabolic, z < 0 hyperbolic). z = 0 is the
    // standard, well-behaved seed for single-rev short-way transfers.
    let sqrt_mu = mu.sqrt();
    let mut z = 0.0_f64;
    let mut residual_seconds = f64::INFINITY;
    let mut converged = false;
    let mut iters = 0;

    while iters < MAX_ITERS {
        let c = stumpff_c(z);
        let s = stumpff_s(z);
        let y = r1n + r2n + a_coef * (z * s - 1.0) / c.sqrt();

        // A negative y means z is below the physical branch for this geometry
        // (short-way single rev has y > 0). Nudge z up and retry rather than
        // taking √(negative); if it never recovers the cap yields NonConvergence.
        if y <= 0.0 {
            z += 0.1;
            iters += 1;
            continue;
        }

        let chi3_c3 = (y / c).powf(1.5) * s;
        let computed_tof = (chi3_c3 + a_coef * y.sqrt()) / sqrt_mu;
        residual_seconds = computed_tof - tof_seconds;
        if residual_seconds.abs() < TOF_REL_TOL * tof_seconds {
            converged = true;
            break;
        }

        // dF/dz for the Newton step (Curtis Algorithm 5.2), with the z→0 limit
        // taken analytically to avoid the 1/(2z) blow-up.
        let dfdz = if z.abs() < 1e-9 {
            let y0 = y;
            std::f64::consts::SQRT_2 / 40.0 * y0.powf(1.5)
                + a_coef / 8.0 * (y0.sqrt() + a_coef * (0.5 / y0).sqrt())
        } else {
            (y / c).powf(1.5)
                * (1.0 / (2.0 * z) * (c - 1.5 * s / c) + 0.75 * s * s / c)
                + a_coef / 8.0 * (3.0 * s / c * y.sqrt() + a_coef * (c / y).sqrt())
        };
        // F(z) = √μ·(computed_tof − tof); its root is the same z. Scale the
        // Newton step by √μ so residual_seconds and dF/dz share units.
        z -= (residual_seconds * sqrt_mu) / dfdz;
        iters += 1;
    }

    if !converged {
        return Err(LambertError::NonConvergence {
            iterations: iters,
            residual_seconds,
        });
    }

    // Lagrange coefficients from the converged z, then the endpoint velocities
    // (Curtis eq. 5.28–5.29).
    let c = stumpff_c(z);
    let s = stumpff_s(z);
    let y = r1n + r2n + a_coef * (z * s - 1.0) / c.sqrt();
    let f = 1.0 - y / r1n;
    let g = a_coef * (y / mu).sqrt();
    let g_dot = 1.0 - y / r2n;

    let v1 = (r2 - f * r1) / g;
    let v2 = (g_dot * r2 - r1) / g;

    Ok(LambertSolution { v1, v2 })
}

/// Stumpff function `C(z)` — the even series `(1 − cos√z)/z` continued to `z ≤ 0`
/// via the hyperbolic form, with a short Taylor series across `z ≈ 0` to dodge
/// the `0/0` cancellation there.
fn stumpff_c(z: f64) -> f64 {
    if z > 1e-6 {
        let s = z.sqrt();
        (1.0 - s.cos()) / z
    } else if z < -1e-6 {
        let s = (-z).sqrt();
        (s.cosh() - 1.0) / (-z)
    } else {
        // C(z) = 1/2 − z/24 + z²/720 − …
        0.5 - z / 24.0 + z * z / 720.0
    }
}

/// Stumpff function `S(z)` — `(√z − sin√z)/√z³` continued to `z ≤ 0`, with the
/// small-`z` series for numerical stability near parabolic.
fn stumpff_s(z: f64) -> f64 {
    if z > 1e-6 {
        let s = z.sqrt();
        (s - s.sin()) / s.powi(3)
    } else if z < -1e-6 {
        let s = (-z).sqrt();
        (s.sinh() - s) / s.powi(3)
    } else {
        // S(z) = 1/6 − z/120 + z²/5040 − …
        1.0 / 6.0 - z / 120.0 + z * z / 5040.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::OrbitalElements;
    use crate::epoch::Epoch;
    use crate::propagator::{KeplerPropagator, Propagator};
    use crate::state::StateVector;

    /// Sun gravitational parameter, SI (m³/s²) — the same representative μ the
    /// element↔state and propagator round-trip tests use.
    const MU_SUN: f64 = 1.327_124_400_18e20;
    /// 1 AU in metres.
    const AU: f64 = 1.495_978_707e11;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    // --- The external anchor: an independent published worked example ---------

    #[test]
    fn reproduces_the_poliastro_vallado_worked_example() {
        // Canonical single-rev Lambert example, digits taken from poliastro's
        // "Revisiting Lambert's problem" docs (an independent Izzo-algorithm
        // implementation — a genuine cross-check of a *different* algorithm, not
        // a recalled magic number). Earth-centric, km / s on the page; converted
        // to SI here. μ = Earth's k = 398600.4418 km³/s².
        let mu_earth = 3.986_004_418e14; // m³/s²
        let r1 = Vector3::new(15_945.34e3, 0.0, 0.0);
        let r2 = Vector3::new(12_214.833_99e3, 10_249.467_31e3, 0.0);
        let tof = 76.0 * 60.0; // 76 minutes

        let sol = lambert_universal(r1, r2, tof, mu_earth, true).unwrap();

        let v1_expected = Vector3::new(2058.925, 2915.956, 0.0); // m/s
        let v2_expected = Vector3::new(-3451.5665, 910.313_54, 0.0);
        // Agreement floors out around ~0.02 m/s (≈1e-5 relative): the page rounds
        // r and v to ~7 sig figs and poliastro's Earth μ differs in its last
        // digits from the value used here. 0.05 m/s is a 2–3× margin over the
        // measured residual and still pins the solve to five significant figures
        // against an independent (Izzo-algorithm) implementation.
        assert!(
            (sol.v1 - v1_expected).norm() < 0.05,
            "v1 {:?} vs {:?}",
            sol.v1,
            v1_expected
        );
        assert!(
            (sol.v2 - v2_expected).norm() < 0.05,
            "v2 {:?} vs {:?}",
            sol.v2,
            v2_expected
        );
    }

    // --- The primary oracle: round-trip against the analytic propagator -------

    /// Build a state, propagate it two-body for `tof`, then confirm Lambert on
    /// the two endpoints recovers the *original* departure velocity and the
    /// *propagated* arrival velocity. Validates against a propagator already
    /// pinned to machine precision, across a spread of orbits and short arcs.
    #[test]
    fn round_trips_against_the_kepler_propagator() {
        // (a, e, i, Ω, ω, ν0) — inclinations < 90° so h·ẑ > 0 (prograde), and
        // short arcs (tof a modest fraction of the period) so Δν < π.
        let orbits = [
            (1.0 * AU, 0.0, 0.0, 0.0, 0.0, 0.0),
            (1.3 * AU, 0.2, 0.4, 1.0, 2.0, 0.5),
            (0.8 * AU, 0.35, 0.9, 2.1, 0.3, 1.2),
            (2.0 * AU, 0.15, 0.2, 4.0, 5.0, 3.0),
        ];
        for &(a, e, i, raan, argp, nu0) in &orbits {
            let elems = OrbitalElements::new(a, e, i, raan, argp, nu0);
            let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
            let period = prop.period();
            let seed = prop.state_at(epoch0()).unwrap();

            for frac in [0.08, 0.18, 0.3] {
                let tof = frac * period;
                let arrival = prop.state_at(epoch0().shifted_by_seconds(tof)).unwrap();

                let sol =
                    lambert_universal(seed.position, arrival.position, tof, MU_SUN, true).unwrap();

                let v1_err = (sol.v1 - seed.velocity).norm() / seed.velocity.norm();
                let v2_err = (sol.v2 - arrival.velocity).norm() / arrival.velocity.norm();
                assert!(
                    v1_err < 1e-8 && v2_err < 1e-8,
                    "round-trip err v1={v1_err:.2e} v2={v2_err:.2e} for a={a:.3e} e={e} frac={frac}"
                );
            }
        }
    }

    // --- Free invariants of the transfer conic (no external oracle needed) ----

    /// Any Lambert solution defines one conic, so its specific energy and its
    /// specific angular momentum must agree at both endpoints — invariants the
    /// solve cannot fake, checked across many geometries with zero reference data.
    #[test]
    fn solution_conserves_energy_and_angular_momentum() {
        let cases = [
            (Vector3::new(1.0 * AU, 0.0, 0.0), Vector3::new(0.2 * AU, 1.1 * AU, 0.1 * AU), 0.22),
            (Vector3::new(1.2 * AU, 0.3 * AU, 0.0), Vector3::new(-0.4 * AU, 1.3 * AU, 0.2 * AU), 0.4),
            (Vector3::new(0.7 * AU, -0.5 * AU, 0.1 * AU), Vector3::new(-1.1 * AU, 0.6 * AU, -0.2 * AU), 0.6),
        ];
        for &(r1, r2, frac_year) in &cases {
            let tof = frac_year * 365.25 * 86400.0;
            let sol = lambert_universal(r1, r2, tof, MU_SUN, true).unwrap();

            let energy1 = 0.5 * sol.v1.norm_squared() - MU_SUN / r1.norm();
            let energy2 = 0.5 * sol.v2.norm_squared() - MU_SUN / r2.norm();
            assert!(
                (energy1 - energy2).abs() / energy1.abs() < 1e-9,
                "energy mismatch {energy1:.6e} vs {energy2:.6e}"
            );

            let h1 = r1.cross(&sol.v1);
            let h2 = r2.cross(&sol.v2);
            // Same vector (magnitude and direction) — one orbital plane.
            assert!(
                (h1 - h2).norm() / h1.norm() < 1e-9,
                "angular momentum mismatch {h1:?} vs {h2:?}"
            );
        }
    }

    // --- The degenerate geometries return errors, not NaN ---------------------

    #[test]
    fn collinear_endpoints_are_degenerate() {
        let r1 = Vector3::new(1.0 * AU, 0.0, 0.0);
        let tof = 0.3 * 365.25 * 86400.0;

        // Δν = π: antiparallel — the 180° singularity.
        let anti = Vector3::new(-1.4 * AU, 0.0, 0.0);
        assert!(matches!(
            lambert_universal(r1, anti, tof, MU_SUN, true),
            Err(LambertError::DegenerateGeometry { .. })
        ));

        // Δν = 0: parallel, same direction — a radial transfer with no plane.
        let para = Vector3::new(1.6 * AU, 0.0, 0.0);
        assert!(matches!(
            lambert_universal(r1, para, tof, MU_SUN, true),
            Err(LambertError::DegenerateGeometry { .. })
        ));
    }

    #[test]
    fn prograde_and_retrograde_give_different_transfers() {
        // A non-coplanar-with-z geometry so the two directions are genuinely
        // distinct conics; both must solve, and to different departure velocities.
        let r1 = Vector3::new(1.0 * AU, 0.1 * AU, 0.0);
        let r2 = Vector3::new(0.2 * AU, 1.2 * AU, 0.0);
        let tof = 0.35 * 365.25 * 86400.0;

        let pro = lambert_universal(r1, r2, tof, MU_SUN, true).unwrap();
        let retro = lambert_universal(r1, r2, tof, MU_SUN, false).unwrap();
        assert!(
            (pro.v1 - retro.v1).norm() / pro.v1.norm() > 1e-3,
            "prograde and retrograde should differ"
        );
    }

    #[test]
    fn rejects_degenerate_input() {
        let r1 = Vector3::new(1.0 * AU, 0.0, 0.0);
        let r2 = Vector3::new(0.0, 1.0 * AU, 0.0);
        let tof = 1e7;
        assert!(matches!(
            lambert_universal(r1, r2, 0.0, MU_SUN, true),
            Err(LambertError::InvalidInput { .. })
        ));
        assert!(matches!(
            lambert_universal(r1, r2, tof, 0.0, true),
            Err(LambertError::InvalidInput { .. })
        ));
        assert!(matches!(
            lambert_universal(Vector3::zeros(), r2, tof, MU_SUN, true),
            Err(LambertError::InvalidInput { .. })
        ));
    }

    #[test]
    fn arrival_state_actually_reaches_r2() {
        // Independent of the round-trip's *velocity* check: take the Lambert
        // departure state, propagate it forward two-body, and confirm it lands on
        // r2 at Δt. Closes the loop through a fresh propagator built from the
        // solved velocity, so a self-consistent-but-wrong v1 cannot pass.
        let r1 = Vector3::new(1.0 * AU, 0.2 * AU, 0.05 * AU);
        let r2 = Vector3::new(-0.3 * AU, 1.1 * AU, -0.1 * AU);
        let tof = 0.4 * 365.25 * 86400.0;
        let sol = lambert_universal(r1, r2, tof, MU_SUN, true).unwrap();

        let departure = StateVector::new(r1, sol.v1);
        let elems = OrbitalElements::from_state(departure, MU_SUN).unwrap();
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let landed = prop.state_at(epoch0().shifted_by_seconds(tof)).unwrap();

        assert!(
            (landed.position - r2).norm() / r2.norm() < 1e-8,
            "propagated arrival {:?} != r2 {:?}",
            landed.position,
            r2
        );
        // And the propagated arrival velocity matches Lambert's v2.
        assert!((landed.velocity - sol.v2).norm() / sol.v2.norm() < 1e-8);
    }
}
