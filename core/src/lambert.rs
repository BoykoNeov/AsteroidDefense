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
    /// No `N`-revolution transfer exists for this geometry and time of flight:
    /// the requested `Δt` is **shorter than the minimum** any `N`-rev conic through
    /// these endpoints can take. Not a failure of the solver — a genuine property
    /// of the geometry (you cannot fit `N` laps into less time than the fastest
    /// `N`-lap conic needs), so the porkchop renders it as a gap for that `N` while
    /// lower `N` may still solve.
    NoSolutionForRevolutions {
        /// The revolution count that has no solution here.
        revolutions: u32,
        /// The shortest time of flight (seconds) any `N`-rev transfer through these
        /// endpoints achieves — the threshold the request fell below.
        minimum_tof_seconds: f64,
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
            LambertError::NoSolutionForRevolutions {
                revolutions,
                minimum_tof_seconds,
            } => write!(
                f,
                "no {revolutions}-revolution transfer for this geometry: the fastest one takes \
                 {minimum_tof_seconds:.3e} s, longer than the requested time of flight"
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

/// Bisection cap for the multi-rev branch solve. Each halving gains a bit, so ~80
/// exhausts an `f64`'s worth of the band; the cap is a backstop that turns a
/// pathological bracket into a clean [`LambertError::NonConvergence`], never a hang.
const MAX_BISECTIONS: u32 = 100;

/// The universal variable at exactly **one complete revolution**, `z = (2π)²`, and
/// therefore the hard ceiling of the single-revolution branch.
///
/// This bound is load-bearing, not cosmetic. On `z ∈ (−∞, 4π²)` the time of flight
/// rises monotonically from 0 to **infinity** — so a single-rev root exists for
/// *every* requested `Δt` — but a Newton step from the `z = 0` seed can overshoot
/// straight past the pole into the 1-revolution band `(4π², 16π²)` and converge
/// happily on a root there. The result looks perfect (it reaches `r2` in the
/// requested time) while being a transfer that laps the Sun on the way: a *real*
/// conic, but not the one the caller asked for, and carrying a different `C3`. In a
/// porkchop that is the worst kind of wrong — a plausible number in a cell labelled
/// something it is not. Clamping the iterate below this ceiling keeps
/// [`lambert_universal`] honest about what it returns; callers who *want* the
/// lapping transfer ask for it explicitly via [`lambert_universal_multirev`].
const SINGLE_REV_Z_MAX: f64 = 4.0 * std::f64::consts::PI * std::f64::consts::PI;

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

    // Transfer angle Δν (resolved into [0, 2π) by the requested direction) and the
    // plane coefficient A, both from the shared helper — see `transfer_geometry`
    // for the direction rule and the 180° degeneracy guard.
    let (_dnu, a_coef) = transfer_geometry(r1, r2, prograde)?;

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
        let mut next = z - (residual_seconds * sqrt_mu) / dfdz;
        // Keep the iterate inside the single-revolution band — and catch a NaN step
        // via `is_finite` rather than a comparison, since every comparison against
        // NaN is false and would read as "in range". On overshoot fall back to a
        // bisection step toward the ceiling, which still converges (T is monotone
        // here) while never crossing into the multi-rev branch. See
        // [`SINGLE_REV_Z_MAX`].
        if !next.is_finite() || next >= SINGLE_REV_Z_MAX {
            next = 0.5 * (z + SINGLE_REV_Z_MAX);
        }
        z = next;
        iters += 1;
    }

    if !converged {
        return Err(LambertError::NonConvergence {
            iterations: iters,
            residual_seconds,
        });
    }

    Ok(velocities_from_z(z, r1, r2, r1n, r2n, a_coef, mu))
}

/// Which of the **two** transfers a given revolution count admits.
///
/// For `N ≥ 1` the time-of-flight curve `T(z)` over that revolution's `z`-band is
/// U-shaped — it diverges at both ends and dips to a minimum in between — so any
/// `Δt` above that minimum is met by *two* distinct conics, one on each side of
/// the dip. Single-revolution transfers have no such pair, which is why this
/// choice only appears on the multi-rev entry point.
///
/// The branches are named for the side of the minimum their universal variable
/// sits on, because that is what the solver actually brackets; which one is
/// cheaper in `C3` is a property of the geometry, measured per case rather than
/// assumed here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MultiRevBranch {
    /// The root **below** the time-of-flight minimum (smaller `z`).
    LowZ,
    /// The root **above** the time-of-flight minimum (larger `z`).
    HighZ,
}

/// Solve Lambert's problem allowing **`revolutions` complete laps** of the
/// attractor before arrival (HANDOFF §8's "multi-rev … drop-in refinement").
///
/// `revolutions = 0` is the single-revolution case and delegates to
/// [`lambert_universal`] (the `branch` argument is then meaningless and ignored,
/// since there is only one root). For `revolutions ≥ 1` the transfer makes that
/// many extra trips around before reaching `r2`, which is what long-time-of-flight
/// porkchop cells physically contain: past roughly one orbital period the *fastest*
/// route between two bodies often is not the lazy single-rev arc but a tighter,
/// faster conic that laps the Sun on the way.
///
/// # Why this needs a different root-finder
/// The single-rev solve Newtons on `z` from a `z = 0` seed and that is enough,
/// because `T(z)` is monotone there. Inside the `N`-rev band
/// `z ∈ ((2Nπ)², (2(N+1)π)²)` it is not: `T` blows up at both edges (the Stumpff
/// `C(z)` vanishes at `z = (2kπ)²`) and has a minimum between them. A Newton
/// iteration seeded anywhere in that band happily walks into the wrong basin or
/// off an edge. So this **brackets** instead: scan the band for the minimum,
/// reject a `Δt` below it as [`LambertError::NoSolutionForRevolutions`] (a real
/// geometric gap, not a solver failure), then bisect on the requested side, where
/// `T` *is* monotone. Slower than Newton and completely robust — the right trade
/// for a grid where a single bad root poisons a heatmap cell.
pub fn lambert_universal_multirev(
    r1: Vector3<f64>,
    r2: Vector3<f64>,
    tof_seconds: f64,
    mu: f64,
    prograde: bool,
    revolutions: u32,
    branch: MultiRevBranch,
) -> Result<LambertSolution, LambertError> {
    if revolutions == 0 {
        return lambert_universal(r1, r2, tof_seconds, mu, prograde);
    }

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

    let (_dnu, a_coef) = transfer_geometry(r1, r2, prograde)?;
    let sqrt_mu = mu.sqrt();

    // The N-rev band. Both edges are poles of T(z) (C(z) = 0 there), so the scan
    // stays strictly inside; `tof_from_z` returns `None` on anything degenerate,
    // which the scan simply skips.
    let n = revolutions as f64;
    let tau = std::f64::consts::TAU;
    let z_lo = (n * tau).powi(2);
    let z_hi = ((n + 1.0) * tau).powi(2);

    // Coarse scan for the minimum. A grid (rather than a derivative search) is
    // deliberate: it is immune to the edge poles and to the `None` gaps, and the
    // band is narrow enough that a few hundred evaluations cost nothing.
    const SCAN_POINTS: usize = 512;
    let mut best: Option<(f64, f64)> = None; // (z, T)
    let mut samples: Vec<(f64, f64)> = Vec::with_capacity(SCAN_POINTS);
    for i in 1..SCAN_POINTS {
        let z = z_lo + (z_hi - z_lo) * (i as f64) / (SCAN_POINTS as f64);
        if let Some(t) = tof_from_z(z, a_coef, r1n, r2n, sqrt_mu) {
            samples.push((z, t));
            if best.is_none_or(|(_, bt)| t < bt) {
                best = Some((z, t));
            }
        }
    }
    let Some((z_min, t_min)) = best else {
        // The whole band evaluated degenerate — no N-rev conic through these
        // endpoints at all. Report it as the same geometric gap.
        return Err(LambertError::NoSolutionForRevolutions {
            revolutions,
            minimum_tof_seconds: f64::INFINITY,
        });
    };

    if tof_seconds < t_min {
        return Err(LambertError::NoSolutionForRevolutions {
            revolutions,
            minimum_tof_seconds: t_min,
        });
    }

    // Bracket the requested root on the chosen side of the minimum. `T` is monotone
    // there, so the outermost sample whose `T` still exceeds `Δt` pairs with `z_min`
    // to bracket it.
    let (mut lo, mut hi) = match branch {
        MultiRevBranch::LowZ => {
            let outer = samples
                .iter()
                .filter(|&&(z, t)| z < z_min && t >= tof_seconds)
                .map(|&(z, _)| z)
                .fold(f64::INFINITY, f64::min);
            if !outer.is_finite() {
                return Err(LambertError::NoSolutionForRevolutions {
                    revolutions,
                    minimum_tof_seconds: t_min,
                });
            }
            (outer, z_min)
        }
        MultiRevBranch::HighZ => {
            let outer = samples
                .iter()
                .filter(|&&(z, t)| z > z_min && t >= tof_seconds)
                .map(|&(z, _)| z)
                .fold(f64::NEG_INFINITY, f64::max);
            if !outer.is_finite() {
                return Err(LambertError::NoSolutionForRevolutions {
                    revolutions,
                    minimum_tof_seconds: t_min,
                });
            }
            (z_min, outer)
        }
    };

    // Bisection: robust, and the band is tiny so ~80 halvings reach the f64 floor.
    let mut z = 0.5 * (lo + hi);
    let mut residual_seconds = f64::INFINITY;
    let mut converged = false;
    for _ in 0..MAX_BISECTIONS {
        z = 0.5 * (lo + hi);
        let Some(t) = tof_from_z(z, a_coef, r1n, r2n, sqrt_mu) else {
            // A degenerate probe inside the bracket: nudge off it rather than
            // aborting — the poles are only at the band edges.
            lo = 0.5 * (lo + z);
            continue;
        };
        residual_seconds = t - tof_seconds;
        if residual_seconds.abs() < TOF_REL_TOL * tof_seconds {
            converged = true;
            break;
        }
        // On the LowZ side T decreases with z; on the HighZ side it increases.
        let too_slow = residual_seconds > 0.0;
        let move_up = match branch {
            MultiRevBranch::LowZ => too_slow,
            MultiRevBranch::HighZ => !too_slow,
        };
        if move_up {
            lo = z;
        } else {
            hi = z;
        }
    }

    if !converged {
        return Err(LambertError::NonConvergence {
            iterations: MAX_BISECTIONS,
            residual_seconds,
        });
    }

    Ok(velocities_from_z(z, r1, r2, r1n, r2n, a_coef, mu))
}

/// The transfer angle `Δν` and the plane coefficient `A` shared by every branch of
/// the solve. Split out so the single-rev Newton and the multi-rev bracket cannot
/// disagree about the geometry they are solving — including the 180° degeneracy,
/// which must be one gap, not two slightly different ones.
fn transfer_geometry(
    r1: Vector3<f64>,
    r2: Vector3<f64>,
    prograde: bool,
) -> Result<(f64, f64), LambertError> {
    let r1n = r1.norm();
    let r2n = r2.norm();
    let cos_dnu = (r1.dot(&r2) / (r1n * r2n)).clamp(-1.0, 1.0);
    let cross_z = r1.x * r2.y - r1.y * r2.x;
    let mut dnu = cos_dnu.acos();
    let take_long_way = if prograde {
        cross_z < 0.0
    } else {
        cross_z >= 0.0
    };
    if take_long_way {
        dnu = std::f64::consts::TAU - dnu;
    }

    let sin_dnu = dnu.sin();
    if sin_dnu.abs() < SIN_DNU_EPS {
        return Err(LambertError::DegenerateGeometry {
            transfer_angle_rad: dnu,
        });
    }
    let a_coef = sin_dnu * (r1n * r2n / (1.0 - dnu.cos())).sqrt();
    Ok((dnu, a_coef))
}

/// Time of flight implied by a universal variable `z`, or `None` where the
/// formulation is degenerate (`C(z) → 0` at the band edges, or a non-positive `y`).
///
/// Returning `Option` rather than erroring is what lets the multi-rev scan step
/// straight over the poles at `z = (2kπ)²` instead of special-casing them.
fn tof_from_z(z: f64, a_coef: f64, r1n: f64, r2n: f64, sqrt_mu: f64) -> Option<f64> {
    // `is_finite` first, so a NaN is rejected by that arm rather than by a
    // comparison (every `>` against NaN is false, which reads as "in range").
    let c = stumpff_c(z);
    if !c.is_finite() || c <= 0.0 {
        return None;
    }
    let s = stumpff_s(z);
    let y = r1n + r2n + a_coef * (z * s - 1.0) / c.sqrt();
    if !y.is_finite() || y <= 0.0 {
        return None;
    }
    let t = ((y / c).powf(1.5) * s + a_coef * y.sqrt()) / sqrt_mu;
    t.is_finite().then_some(t)
}

/// Lagrange coefficients from a converged `z`, then the endpoint velocities
/// (Curtis eq. 5.28–5.29). Shared by both entry points so a multi-rev root is
/// turned into velocities by exactly the same algebra as a single-rev one.
fn velocities_from_z(
    z: f64,
    r1: Vector3<f64>,
    r2: Vector3<f64>,
    r1n: f64,
    r2n: f64,
    a_coef: f64,
    mu: f64,
) -> LambertSolution {
    let c = stumpff_c(z);
    let s = stumpff_s(z);
    let y = r1n + r2n + a_coef * (z * s - 1.0) / c.sqrt();
    let f = 1.0 - y / r1n;
    let g = a_coef * (y / mu).sqrt();
    let g_dot = 1.0 - y / r2n;

    LambertSolution {
        v1: (r2 - f * r1) / g,
        v2: (g_dot * r2 - r1) / g,
    }
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

    // --- Multi-revolution transfers ------------------------------------------

    /// A representative multi-rev geometry: two points on a 1 AU-ish plane,
    /// separated by a decent transfer angle, with a time of flight long enough for
    /// a lap. Returned as `(r1, r2, tof)`.
    fn multirev_case() -> (Vector3<f64>, Vector3<f64>, f64) {
        let year = 365.25 * 86400.0;
        (
            Vector3::new(1.0 * AU, 0.0, 0.0),
            Vector3::new(-0.3 * AU, 1.25 * AU, 0.05 * AU),
            2.6 * year,
        )
    }

    /// **The discriminating test.** A multi-rev root is only correct if the conic
    /// it describes actually *goes* from `r1` to `r2` in the requested time — and
    /// that is precisely what a wrong root (the other basin, or an edge artefact)
    /// would fail. So each branch's departure state is handed to the analytic
    /// Kepler propagator and flown for the full time of flight; it must arrive at
    /// `r2`.
    ///
    /// Note this is a genuinely independent check: the solver works in universal
    /// variables and Stumpff functions, the propagator in classical elements and
    /// Kepler's equation. Agreement across that gap is not two spellings of one
    /// formula.
    #[test]
    fn both_multirev_branches_actually_reach_r2() {
        let (r1, r2, tof) = multirev_case();
        for branch in [MultiRevBranch::LowZ, MultiRevBranch::HighZ] {
            let sol = lambert_universal_multirev(r1, r2, tof, MU_SUN, true, 1, branch)
                .unwrap_or_else(|e| panic!("{branch:?} should solve: {e}"));

            let elems = OrbitalElements::from_state(StateVector::new(r1, sol.v1), MU_SUN)
                .expect("departure state is a valid conic");
            let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).expect("propagator");
            let arrived = prop
                .state_at(epoch0().shifted_by_seconds(tof))
                .expect("propagate the transfer");

            let miss = (arrived.position - r2).norm();
            println!(
                "{branch:?}: |v1| = {:.3} km/s, a = {:.4} AU, arrival miss {:.3e} m",
                sol.v1.norm() / 1e3,
                elems.semi_major_axis / AU,
                miss
            );
            assert!(
                miss < 1e-6 * r2.norm(),
                "{branch:?} transfer missed r2 by {miss:.3e} m — the root is not this geometry's"
            );

            // And the arrival velocity the solver reports is the one the conic has.
            let v2_err = (arrived.velocity - sol.v2).norm() / sol.v2.norm();
            assert!(v2_err < 1e-8, "{branch:?} arrival velocity error {v2_err:.2e}");
        }
    }

    /// The two branches are genuinely *different* transfers, not one root found
    /// twice. Without this, a solver that ignored `branch` and returned the same
    /// conic each time would pass the reach-`r2` test above.
    #[test]
    fn the_two_branches_are_distinct_transfers() {
        let (r1, r2, tof) = multirev_case();
        let lo = lambert_universal_multirev(r1, r2, tof, MU_SUN, true, 1, MultiRevBranch::LowZ)
            .expect("LowZ solves");
        let hi = lambert_universal_multirev(r1, r2, tof, MU_SUN, true, 1, MultiRevBranch::HighZ)
            .expect("HighZ solves");
        let separation = (lo.v1 - hi.v1).norm() / lo.v1.norm();
        println!("branch separation in departure velocity: {:.3}%", separation * 100.0);
        assert!(
            separation > 1e-3,
            "the two branches should be distinct conics, got a relative separation of {separation:.2e}"
        );
    }

    /// Orbital period of the conic a Lambert departure state lies on.
    fn transfer_period(r1: Vector3<f64>, v1: Vector3<f64>) -> f64 {
        // vis-viva for a: 1/a = 2/r − v²/μ
        let a = 1.0 / (2.0 / r1.norm() - v1.norm_squared() / MU_SUN);
        std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt()
    }

    /// **The regression that pins `SINGLE_REV_Z_MAX`.** A long time of flight used
    /// to make the single-rev Newton overshoot the `z = 4π²` pole and converge in
    /// the 1-revolution band — returning a conic that laps the Sun, labelled
    /// single-rev, with a `C3` that belonged to a different transfer.
    ///
    /// The check is physical rather than a peek at `z`: a transfer that completes
    /// **fewer than one revolution** must take *less* than its own orbital period,
    /// and a 1-rev transfer must take *more*. Both halves are asserted, so the test
    /// distinguishes "single-rev solver stayed single-rev" from "multi-rev solver
    /// actually laps" instead of merely observing that the two now differ.
    #[test]
    fn the_single_rev_solver_never_returns_a_lapping_transfer() {
        let (r1, r2, tof) = multirev_case();

        let single = lambert_universal(r1, r2, tof, MU_SUN, true).expect("single-rev solves");
        let single_period = transfer_period(r1, single.v1);
        println!(
            "single-rev: tof {:.3} yr vs transfer period {:.3} yr",
            tof / (365.25 * 86400.0),
            single_period / (365.25 * 86400.0)
        );
        assert!(
            tof < single_period,
            "a single-rev transfer must finish inside one of its own periods \
             (tof {tof:.4e} s, period {single_period:.4e} s) — the solver crossed into the \
             multi-rev band"
        );

        let multi = lambert_universal_multirev(r1, r2, tof, MU_SUN, true, 1, MultiRevBranch::LowZ)
            .expect("1-rev solves");
        let multi_period = transfer_period(r1, multi.v1);
        assert!(
            tof > multi_period,
            "a 1-rev transfer must take longer than one of its own periods \
             (tof {tof:.4e} s, period {multi_period:.4e} s)"
        );

        // And with the bands now separated, the two are substantially different
        // transfers rather than the same root reached two ways.
        let d = (single.v1 - multi.v1).norm() / single.v1.norm();
        assert!(
            d > 1e-2,
            "single-rev and 1-rev transfers over the same span should differ, got {d:.2e}"
        );
    }

    /// The clamp's own worst case. At 2.6 yr the single-rev root sits comfortably
    /// mid-band and Newton reaches it in a few steps. Push the time of flight far
    /// out and the root migrates toward the `z = 4π²` pole, where every Newton step
    /// overshoots and the clamp degenerates to **pure bisection** toward the
    /// ceiling — so this is the case that actually tests whether the fallback
    /// converges inside `MAX_ITERS` rather than quietly returning
    /// `NonConvergence` on any long window.
    #[test]
    fn the_single_rev_clamp_still_converges_for_very_long_windows() {
        let (r1, r2, _) = multirev_case();
        let year = 365.25 * 86400.0;
        for years in [5.0, 10.0, 20.0, 50.0] {
            let tof = years * year;
            let sol = lambert_universal(r1, r2, tof, MU_SUN, true)
                .unwrap_or_else(|e| panic!("{years} yr window should converge: {e}"));
            let period = transfer_period(r1, sol.v1);
            assert!(
                period.is_finite() && tof < period,
                "{years} yr: still must be a sub-one-revolution transfer \
                 (period {:.3} yr)",
                period / year
            );
            // And it is a real transfer: it reaches r2.
            let elems = OrbitalElements::from_state(StateVector::new(r1, sol.v1), MU_SUN)
                .expect("valid conic");
            let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).expect("propagator");
            let arrived = prop
                .state_at(epoch0().shifted_by_seconds(tof))
                .expect("propagate");
            let miss = (arrived.position - r2).norm();
            assert!(
                miss < 1e-6 * r2.norm(),
                "{years} yr transfer missed r2 by {miss:.3e} m"
            );
        }
    }

    /// `revolutions = 0` is exactly the single-rev entry point — the delegation the
    /// mission layer relies on when it sweeps `N = 0, 1, 2 …` uniformly.
    #[test]
    fn zero_revolutions_delegates_to_the_single_rev_solver() {
        let r1 = Vector3::new(1.0 * AU, 0.0, 0.0);
        let r2 = Vector3::new(0.2 * AU, 1.1 * AU, 0.1 * AU);
        let tof = 0.22 * 365.25 * 86400.0;
        let direct = lambert_universal(r1, r2, tof, MU_SUN, true).unwrap();
        for branch in [MultiRevBranch::LowZ, MultiRevBranch::HighZ] {
            let via = lambert_universal_multirev(r1, r2, tof, MU_SUN, true, 0, branch).unwrap();
            assert_eq!(via, direct, "N=0 must be bit-identical to the single-rev solve");
        }
    }

    /// Asking for more laps than the time allows is a **geometric gap**, reported as
    /// such with the threshold that was missed — never a `NaN`, and never a
    /// non-convergence that reads like a solver bug. This is the multi-rev sibling
    /// of the 180° `DegenerateGeometry` discipline.
    #[test]
    fn a_time_of_flight_below_the_minimum_is_a_named_gap() {
        let (r1, r2, _) = multirev_case();
        // Far too short for even one extra lap at this scale.
        let tof = 0.3 * 365.25 * 86400.0;
        match lambert_universal_multirev(r1, r2, tof, MU_SUN, true, 1, MultiRevBranch::LowZ) {
            Err(LambertError::NoSolutionForRevolutions {
                revolutions,
                minimum_tof_seconds,
            }) => {
                assert_eq!(revolutions, 1);
                assert!(
                    minimum_tof_seconds.is_finite() && minimum_tof_seconds > tof,
                    "the reported minimum {minimum_tof_seconds:.3e} s must exceed the request"
                );
            }
            other => panic!("expected a NoSolutionForRevolutions gap, got {other:?}"),
        }
    }

    /// More laps take more time: the minimum time of flight reported for `N = 2`
    /// must exceed the one for `N = 1`. Pins that the band actually tracks `N`
    /// rather than the solver quietly searching the same interval every time —
    /// which the reach-`r2` test alone would not catch, since a 1-rev root does
    /// reach `r2`.
    #[test]
    fn each_extra_revolution_raises_the_minimum_time_of_flight() {
        let (r1, r2, _) = multirev_case();
        let short = 0.05 * 365.25 * 86400.0;
        let minimum_for = |n: u32| {
            match lambert_universal_multirev(r1, r2, short, MU_SUN, true, n, MultiRevBranch::LowZ) {
                Err(LambertError::NoSolutionForRevolutions {
                    minimum_tof_seconds, ..
                }) => minimum_tof_seconds,
                other => panic!("expected a gap for N={n}, got {other:?}"),
            }
        };
        let (m1, m2, m3) = (minimum_for(1), minimum_for(2), minimum_for(3));
        println!(
            "minimum TOF: 1 rev {:.3} yr, 2 rev {:.3} yr, 3 rev {:.3} yr",
            m1 / (365.25 * 86400.0),
            m2 / (365.25 * 86400.0),
            m3 / (365.25 * 86400.0),
        );
        assert!(
            m1 < m2 && m2 < m3,
            "minimum time of flight must grow with revolutions: {m1:.3e} / {m2:.3e} / {m3:.3e}"
        );
    }

    /// The round-trip oracle extended to multiple laps: fly a real orbit for more
    /// than one full period, then confirm a 1-rev Lambert solve recovers *that*
    /// orbit's departure velocity on one of its two branches. Which branch it lands
    /// on is a property of the geometry, so it is discovered and printed rather
    /// than asserted — but that the true orbit is among the two roots is the claim.
    #[test]
    fn multirev_recovers_a_genuine_multi_lap_orbit() {
        let elems = OrbitalElements::new(1.3 * AU, 0.2, 0.4, 1.0, 2.0, 0.5);
        let prop = KeplerPropagator::new(elems, MU_SUN, epoch0()).unwrap();
        let period = prop.period();
        let seed = prop.state_at(epoch0()).unwrap();

        // 1.7 periods: one complete lap plus a direct arc — an N = 1 transfer.
        let tof = 1.7 * period;
        let arrival = prop.state_at(epoch0().shifted_by_seconds(tof)).unwrap();

        let mut best: Option<(MultiRevBranch, f64)> = None;
        for branch in [MultiRevBranch::LowZ, MultiRevBranch::HighZ] {
            if let Ok(sol) =
                lambert_universal_multirev(seed.position, arrival.position, tof, MU_SUN, true, 1, branch)
            {
                let err = (sol.v1 - seed.velocity).norm() / seed.velocity.norm();
                if best.is_none_or(|(_, b)| err < b) {
                    best = Some((branch, err));
                }
            }
        }
        let (branch, err) = best.expect("at least one branch should solve");
        println!("the true 1.7-period orbit is the {branch:?} branch (rel. error {err:.2e})");
        assert!(
            err < 1e-7,
            "no branch recovered the true multi-lap departure velocity (best {err:.2e} on {branch:?})"
        );
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
