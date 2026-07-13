//! `Integrator` — a swappable ODE stepper for the equation of motion (HANDOFF §4).
//!
//! The integrated body obeys a second-order ODE, `r̈ = a(t, r, ṙ)`, with the
//! acceleration supplied by a [`ForceModel`](crate::forces::ForceModel). An
//! [`Integrator`] advances a [`StateVector`] `(r, ṙ)` by one step of `Δt`; the
//! trait exists so RK4 / dop853 / (Tier-2) IAS15 / symplectic steppers are
//! interchangeable behind one interface (§4, §5). It is deliberately
//! **object-safe** — no generics, no `Self` in the signature — matching the
//! [`Propagator`](crate::propagator::Propagator) convention, so a stepper can be
//! chosen at run time via `&dyn Integrator`.
//!
//! # This task (§10.7): RK4 first, then dop853
//! [`Rk4`] is the classical fixed-step fourth-order Runge–Kutta method. It came
//! first specifically to **exercise the free-invariant tests**: unlike the
//! analytic Kepler map (which conserves by construction), a numerical stepper
//! *drifts*, so it is what forced the invariant harness to grow its
//! error-growth-rate / convergence-order assertion shape (HANDOFF §6, §10.5).
//!
//! [`Dop853`] is the **MVP encounter integrator** (adaptive, 8th-order): the
//! Dormand–Prince 8(5,3) pair with Hairer's combined error norm. It honours the
//! *same* object-safe [`Integrator`] trait — a `step(dt)` call sub-steps
//! adaptively *inside* the interval under its own error control, which is exactly
//! the resolved architecture "fixed snapshot **cadence**, adaptive integration
//! **step** between snapshots" (HANDOFF §2). Its **dense output**
//! ([`Dop853::step_dense`] → [`DenseSegment`], §10.9) is the continuous extension
//! the [`clock`](crate::clock) samples between snapshots, so a sub-snapshot query
//! interpolates through the encounter's curvature instead of linearly across it.
//!
//! # Frame
//! The stepper is frame-agnostic — it advances whatever frame the force model and
//! seed state are expressed in. The core integrates in **barycentric ICRF, SI**
//! (HANDOFF §5); the force model enforces that, the integrator just steps.

use crate::epoch::Epoch;
use crate::forces::{ForceError, ForceModel};
use crate::state::StateVector;

/// The 12 step-stage derivatives of one DOP853 sub-step (`kᵣ` = stage velocity or
/// `kᵥ` = stage acceleration). Named so the stage-carrying signatures stay legible.
type StageDerivs = [nalgebra::Vector3<f64>; 12];

/// Failure modes of a step. A fixed step ([`Rk4`]) can only fail because the
/// force model failed to evaluate; an adaptive step ([`Dop853`]) can additionally
/// give up when its error controller cannot make progress. The enum is kept
/// distinct from [`ForceError`] so these adaptive modes fit without a breaking
/// change (as the batch-1 doc anticipated).
#[derive(Debug, Clone, PartialEq)]
pub enum IntegratorError {
    /// The force model could not be evaluated at a stage of the step.
    Force(ForceError),
    /// An adaptive stepper's step size shrank below the floor for the current
    /// epoch (repeated rejections could not meet the tolerance) — fail loud
    /// rather than spin forever on a step that cannot be accepted.
    StepSizeUnderflow {
        /// The epoch (seconds past J2000, TDB) where the step stalled.
        epoch_seconds: f64,
        /// The rejected step size (seconds) at the point of underflow.
        step_seconds: f64,
    },
    /// An adaptive stepper exceeded its sub-step budget while sub-stepping across
    /// one `step` call — a runaway backstop, not a normal outcome.
    MaxStepsExceeded {
        /// The configured sub-step ceiling that was hit.
        limit: u64,
    },
}

impl std::fmt::Display for IntegratorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IntegratorError::Force(e) => write!(f, "integrator force evaluation failed: {e}"),
            IntegratorError::StepSizeUnderflow {
                epoch_seconds,
                step_seconds,
            } => write!(
                f,
                "adaptive step underflowed at t={epoch_seconds:.6} s (J2000 TDB): \
                 step {step_seconds:.3e} s fell below the floor without meeting tolerance"
            ),
            IntegratorError::MaxStepsExceeded { limit } => write!(
                f,
                "adaptive integrator exceeded its sub-step budget ({limit}) within one step"
            ),
        }
    }
}

impl std::error::Error for IntegratorError {}

impl From<ForceError> for IntegratorError {
    fn from(e: ForceError) -> Self {
        IntegratorError::Force(e)
    }
}

/// A single-step ODE integrator for `r̈ = a(t, r, ṙ)` (HANDOFF §4). Object-safe.
pub trait Integrator {
    /// Advance `state` (at `epoch`) by `dt` seconds under `force`, returning the
    /// state at `epoch + dt`. `dt` may be negative (backward integration).
    fn step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
    ) -> Result<StateVector, IntegratorError>;
}

/// Classical fixed-step fourth-order Runge–Kutta (RK4).
///
/// Applied to the first-order system `y = (r, ṙ)`, `ẏ = (ṙ, a(t, r, ṙ))`. The
/// four stages evaluate the force at `t`, `t + Δt/2` (twice), and `t + Δt`, so a
/// **time-varying** field (moving perturbers) is sampled at the correct
/// sub-step epochs — carrying the epoch through each stage is load-bearing, not
/// cosmetic (a stepper that evaluated every stage at `t` would silently drop to
/// first order on a non-autonomous field). Global error is `O(Δt⁴)`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Rk4;

impl Integrator for Rk4 {
    fn step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
    ) -> Result<StateVector, IntegratorError> {
        let h = dt;
        let r = state.position;
        let v = state.velocity;

        let epoch_mid = epoch.shifted_by_seconds(0.5 * h);
        let epoch_end = epoch.shifted_by_seconds(h);

        // Stage 1 — slope at the start.
        let a1 = force.acceleration(epoch, state)?;

        // Stage 2 — slope at the midpoint, using the stage-1 slope.
        let s2 = StateVector::new(r + 0.5 * h * v, v + 0.5 * h * a1);
        let a2 = force.acceleration(epoch_mid, &s2)?;

        // Stage 3 — slope at the midpoint, using the stage-2 slope.
        let s3 = StateVector::new(r + 0.5 * h * s2.velocity, v + 0.5 * h * a2);
        let a3 = force.acceleration(epoch_mid, &s3)?;

        // Stage 4 — slope at the endpoint, using the stage-3 slope.
        let s4 = StateVector::new(r + h * s3.velocity, v + h * a3);
        let a4 = force.acceleration(epoch_end, &s4)?;

        // Weighted average of the four slopes (the position-derivatives are the
        // stage velocities; the velocity-derivatives are the stage accelerations).
        let dr = (v + 2.0 * s2.velocity + 2.0 * s3.velocity + s4.velocity) / 6.0;
        let dv = (a1 + 2.0 * a2 + 2.0 * a3 + a4) / 6.0;

        Ok(StateVector::new(r + h * dr, v + h * dv))
    }
}

/// Dormand–Prince 8(5,3) adaptive Runge–Kutta — the **MVP encounter integrator**
/// (HANDOFF §5, §7, §10.7).
///
/// An 8th-order explicit RK pair (the classical "DOP853" of Hairer, Nørsett &
/// Wanner) with an embedded 5th- and 3rd-order error estimate. It advances the
/// second-order equation of motion `r̈ = a(t, r, ṙ)` written as the first-order
/// system `ẏ = (ṙ, a)`, `y = (r, ṙ)`.
///
/// # Adaptive stepping under the fixed `Integrator` trait
/// [`Integrator::step`] asks for one advance of exactly `dt`. `Dop853` honours
/// that by **sub-stepping adaptively across `[epoch, epoch + dt]`** under its own
/// local-error control, landing the final sub-step *exactly* on `epoch + dt`.
/// This is the resolved architecture — *fixed snapshot cadence, adaptive
/// integration step between snapshots* (HANDOFF §2): the clock will call
/// `step(cadence)`; the stepper picks its own internal steps within it.
///
/// # Pure / deterministic
/// `step` takes `&self` and carries **no** state across calls: each call
/// estimates its own initial step (Hairer's automatic algorithm) and sub-steps
/// from scratch. Warm-starting the step size across snapshots is a possible later
/// optimisation, not needed for correctness — and keeping `step` pure preserves
/// same-build-same-output determinism (HANDOFF §2).
///
/// # Error control
/// Local error uses **Hairer's combined 5(3) norm** (not a naive `y₈ − y₅`): the
/// 5th- and 3rd-order embedded estimates are blended so the controller is robust
/// where either alone would misjudge the step. The step is accepted when that
/// norm is `< 1` against the per-component tolerance `atol + rtol·max(|yᵢ|,
/// |yᵢ,new|)`; the next step size follows the standard `SAFETY · err^(−1/8)`
/// rule, clamped to `[MIN_FACTOR, MAX_FACTOR]`. Backward integration (`dt < 0`)
/// flips the direction of every step and target comparison; forward-back
/// reversibility is a test invariant.
///
/// # Coefficients
/// The Butcher tableau (`A`, `B`, `C`) and the error weights (`E5`, `E3`) are
/// transcribed from SciPy's `scipy/integrate/_ivp/dop853_coefficients.py` — a
/// clean, machine-readable copy of Hairer's published constants — and
/// cross-checked by the tableau's own consistency conditions in the tests
/// (`Σⱼ A[i][j] = C[i]`, `Σ B = 1`, `Σ E = 0`). Because the FSAL derivative's error
/// weight is zero (`E5[12] = E3[12] = 0`), one accepted step costs 12 force
/// evaluations (11 interior stages + the next step's start derivative).
///
/// # Dense output (§10.9)
/// [`Dop853::step_dense`] additionally emits the **continuous extension** the
/// clock samples between snapshots: a degree-7 interpolant per accepted sub-step
/// ([`DenseSegment`]). It reuses the 12 step stages and the FSAL derivative and
/// evaluates **3 more** stages (`C_EXTRA`/`A_EXTRA` in the tableau), so a recorded
/// step costs 3 force evaluations beyond a plain one — paid only on the
/// dense path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dop853 {
    /// Relative tolerance (per state component). Default `1e-9`.
    rtol: f64,
    /// Absolute tolerance (per state component, in the flat `(m, m/s)` state).
    /// Default `1e-9`; guards components passing through zero.
    atol: f64,
    /// Optional cap on the absolute sub-step size (seconds). `None` = unbounded.
    max_step: Option<f64>,
    /// Runaway backstop: the maximum number of accepted sub-steps within one
    /// `step` call before failing loud. Default `1_000_000`.
    max_substeps: u64,
}

/// Controller safety factor (Hairer/SciPy `SAFETY`).
const DOP_SAFETY: f64 = 0.9;
/// Minimum step-shrink factor on rejection (SciPy `MIN_FACTOR`).
const DOP_MIN_FACTOR: f64 = 0.2;
/// Maximum step-growth factor on acceptance (SciPy `MAX_FACTOR`).
const DOP_MAX_FACTOR: f64 = 10.0;
/// Step-size exponent `−1/(error_estimator_order + 1) = −1/8` (7th-order
/// embedded estimate).
const DOP_ERR_EXPONENT: f64 = -0.125;

impl Default for Dop853 {
    fn default() -> Self {
        Self::new()
    }
}

impl Dop853 {
    /// A `Dop853` with default tolerances (`rtol = atol = 1e-9`), no step cap,
    /// and a `1_000_000` sub-step runaway backstop.
    pub fn new() -> Self {
        Self {
            rtol: 1e-9,
            atol: 1e-9,
            max_step: None,
            max_substeps: 1_000_000,
        }
    }

    /// Set the relative tolerance (per component). Panics if not finite and > 0.
    pub fn with_rtol(mut self, rtol: f64) -> Self {
        assert!(
            rtol.is_finite() && rtol > 0.0,
            "rtol must be finite and > 0"
        );
        self.rtol = rtol;
        self
    }

    /// Set the absolute tolerance (per component). Panics if not finite and > 0.
    pub fn with_atol(mut self, atol: f64) -> Self {
        assert!(
            atol.is_finite() && atol > 0.0,
            "atol must be finite and > 0"
        );
        self.atol = atol;
        self
    }

    /// Set both tolerances at once.
    pub fn with_tolerances(self, rtol: f64, atol: f64) -> Self {
        self.with_rtol(rtol).with_atol(atol)
    }

    /// Cap the absolute sub-step size at `max_step` seconds. Panics if not
    /// finite and > 0.
    pub fn with_max_step(mut self, max_step: f64) -> Self {
        assert!(
            max_step.is_finite() && max_step > 0.0,
            "max_step must be finite and > 0"
        );
        self.max_step = Some(max_step);
        self
    }

    /// Set the sub-step runaway backstop. Panics if zero.
    pub fn with_max_substeps(mut self, max_substeps: u64) -> Self {
        assert!(max_substeps >= 1, "max_substeps must be at least 1");
        self.max_substeps = max_substeps;
        self
    }

    /// The relative tolerance in force.
    pub fn rtol(&self) -> f64 {
        self.rtol
    }

    /// The absolute tolerance in force.
    pub fn atol(&self) -> f64 {
        self.atol
    }

    /// Per-component error scale `atol + max(|a|, |b|)·rtol` for a `Vector3` pair.
    fn scale(
        &self,
        a: nalgebra::Vector3<f64>,
        b: nalgebra::Vector3<f64>,
    ) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(
            self.atol + a.x.abs().max(b.x.abs()) * self.rtol,
            self.atol + a.y.abs().max(b.y.abs()) * self.rtol,
            self.atol + a.z.abs().max(b.z.abs()) * self.rtol,
        )
    }

    /// Hairer's automatic initial step size (SciPy `select_initial_step`), using
    /// the already-computed start derivative `f0 = (f0r, f0v)`. `interval_len` is
    /// `|dt|`; `direction` is `±1`.
    #[allow(clippy::too_many_arguments)]
    fn initial_step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        f0r: nalgebra::Vector3<f64>,
        f0v: nalgebra::Vector3<f64>,
        direction: f64,
        interval_len: f64,
    ) -> Result<f64, IntegratorError> {
        if interval_len == 0.0 {
            return Ok(0.0);
        }
        let r = state.position;
        let v = state.velocity;
        // Scale from |y0| only (matching SciPy's initial-step routine), reused for
        // y0, f0, and (f1 − f0).
        let sr = self.scale(r, r);
        let sv = self.scale(v, v);
        let d0 = rms_scaled(r, v, sr, sv);
        let d1 = rms_scaled(f0r, f0v, sr, sv);
        let mut h0 = if d0 < 1e-5 || d1 < 1e-5 {
            1e-6
        } else {
            0.01 * d0 / d1
        };
        h0 = h0.min(interval_len);

        let y1 = StateVector::new(r + h0 * direction * f0r, v + h0 * direction * f0v);
        let (f1r, f1v) = derivative(force, epoch.shifted_by_seconds(h0 * direction), &y1)?;
        let d2 = rms_scaled(f1r - f0r, f1v - f0v, sr, sv) / h0;

        // Error-estimator order for DOP853 is 7.
        let h1 = if d1 <= 1e-15 && d2 <= 1e-15 {
            (1e-6_f64).max(h0 * 1e-3)
        } else {
            (0.01 / d1.max(d2)).powf(1.0 / (7.0 + 1.0))
        };

        let max_step = self.max_step.unwrap_or(f64::INFINITY);
        Ok((100.0 * h0).min(h1).min(interval_len).min(max_step))
    }

    /// Attempt one signed sub-step of `h` seconds from `state` at `epoch`, given
    /// the start derivative `k0 = (k0r, k0v)` (= `f(epoch, state)`, reused as the
    /// first RK stage). Returns the 8th-order solution, the Hairer 5(3) error
    /// norm, and the 12 stage derivatives `(kr, kv)` — the accept/reject decision
    /// stays in [`Self::integrate`], and the stage arrays let the dense-output
    /// path ([`Self::step_dense`]) build its interpolant without recomputing them.
    #[allow(clippy::too_many_arguments)]
    fn attempt_step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        k0r: nalgebra::Vector3<f64>,
        k0v: nalgebra::Vector3<f64>,
        h: f64,
    ) -> Result<(StateVector, f64, StageDerivs, StageDerivs), IntegratorError> {
        use dop853_tableau::{A, B, C, E3, E5};
        use nalgebra::Vector3;

        let r = state.position;
        let v = state.velocity;

        // Stage derivatives kᵣ (= stage velocity) and kᵥ (= stage acceleration).
        let mut kr = [Vector3::zeros(); 12];
        let mut kv = [Vector3::zeros(); 12];
        kr[0] = k0r;
        kv[0] = k0v;

        for s in 1..12 {
            let mut dr = Vector3::zeros();
            let mut dv = Vector3::zeros();
            for j in 0..s {
                let a = A[s][j];
                if a != 0.0 {
                    dr += a * kr[j];
                    dv += a * kv[j];
                }
            }
            let stage = StateVector::new(r + h * dr, v + h * dv);
            let acc = force.acceleration(epoch.shifted_by_seconds(C[s] * h), &stage)?;
            kr[s] = stage.velocity;
            kv[s] = acc;
        }

        // 8th-order solution: y_new = y + h · Σ B[s]·k[s].
        let mut sr = Vector3::zeros();
        let mut sv = Vector3::zeros();
        for s in 0..12 {
            sr += B[s] * kr[s];
            sv += B[s] * kv[s];
        }
        let new = StateVector::new(r + h * sr, v + h * sv);

        // Embedded 5th- and 3rd-order error vectors (E5[12] = E3[12] = 0, so the
        // uncomputed FSAL stage contributes nothing — the loop stops at 12).
        let mut e5r = Vector3::zeros();
        let mut e5v = Vector3::zeros();
        let mut e3r = Vector3::zeros();
        let mut e3v = Vector3::zeros();
        for s in 0..12 {
            e5r += E5[s] * kr[s];
            e5v += E5[s] * kv[s];
            e3r += E3[s] * kr[s];
            e3v += E3[s] * kv[s];
        }

        let scale_r = self.scale(r, new.position);
        let scale_v = self.scale(v, new.velocity);
        let err5_2 =
            e5r.component_div(&scale_r).norm_squared() + e5v.component_div(&scale_v).norm_squared();
        let err3_2 =
            e3r.component_div(&scale_r).norm_squared() + e3v.component_div(&scale_v).norm_squared();

        // Hairer's combined 5(3) error norm: |h| · err5² / √((err5² + 0.01·err3²)·n).
        let error_norm = if err5_2 == 0.0 && err3_2 == 0.0 {
            0.0
        } else {
            let denom = err5_2 + 0.01 * err3_2;
            h.abs() * err5_2 / (denom * 6.0).sqrt()
        };

        Ok((new, error_norm, kr, kv))
    }

    /// The shared adaptive driver behind both [`Integrator::step`] and
    /// [`Self::step_dense`]. Sub-steps across `[epoch, epoch + dt]` under the
    /// error controller (§ struct docs), landing the final sub-step exactly on
    /// `epoch + dt`, and returns the state there.
    ///
    /// `on_accept` is invoked once per **accepted** sub-step, in integration
    /// order, with everything the dense-output interpolant needs:
    /// `(t_offset, h, y_before, y_after, kr, kv, fsal_r, fsal_v)` — the sub-step's
    /// start offset from `epoch` (seconds) and signed length, the states at its
    /// two ends, the 12 step stages, and the FSAL derivative `K[12]` at the end
    /// (which this loop computes anyway as the next sub-step's first stage). The
    /// plain `step` passes a no-op; `step_dense` builds a [`DenseSegment`]. Keeping
    /// one loop means the two paths cannot drift in their accept/reject or
    /// endpoint-clamping logic.
    #[allow(clippy::too_many_arguments)]
    fn integrate(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
        mut on_accept: impl FnMut(
            f64,
            f64,
            &StateVector,
            &StateVector,
            &StageDerivs,
            &StageDerivs,
            nalgebra::Vector3<f64>,
            nalgebra::Vector3<f64>,
        ) -> Result<(), IntegratorError>,
    ) -> Result<StateVector, IntegratorError> {
        if dt == 0.0 {
            return Ok(*state);
        }
        let direction = dt.signum();
        let interval_len = dt.abs();
        // Track the offset from `epoch` (not an absolute second count) so the
        // sub-step epochs keep hifitime's full precision, matching Rk4.
        let t0_abs = epoch.tdb_seconds_past_j2000();
        let mut t = 0.0_f64; // offset from `epoch`, in seconds
        let t_bound = dt; // offset of the target
        let mut y = *state;

        // Start derivative (reused as the first RK stage of the first sub-step).
        let (mut fr, mut fv) = derivative(force, epoch, &y)?;
        let mut h_abs = self.initial_step(force, epoch, &y, fr, fv, direction, interval_len)?;
        let max_step = self.max_step.unwrap_or(f64::INFINITY);

        let mut substeps = 0_u64;
        while (t - t_bound) * direction < 0.0 {
            substeps += 1;
            if substeps > self.max_substeps {
                return Err(IntegratorError::MaxStepsExceeded {
                    limit: self.max_substeps,
                });
            }
            // Floor on the step size, scaled to the current absolute time so it
            // tracks the local ulp (prevents an unproductive shrink-to-zero spin).
            let min_step = 10.0 * f64::EPSILON * (t0_abs + t).abs().max(1.0);
            if h_abs > max_step {
                h_abs = max_step;
            }

            let mut step_rejected = false;
            loop {
                if h_abs < min_step {
                    return Err(IntegratorError::StepSizeUnderflow {
                        epoch_seconds: t0_abs + t,
                        step_seconds: h_abs * direction,
                    });
                }
                // Propose a signed step, then clamp the endpoint exactly onto
                // t_bound so the final sub-step never overshoots the snapshot.
                let mut h = h_abs * direction;
                let mut t_new = t + h;
                if (t_new - t_bound) * direction > 0.0 {
                    t_new = t_bound;
                }
                h = t_new - t;
                h_abs = h.abs();

                let step_epoch = epoch.shifted_by_seconds(t);
                let (y_new, error_norm, kr, kv) =
                    self.attempt_step(force, step_epoch, &y, fr, fv, h)?;

                if error_norm < 1.0 {
                    let raw = if error_norm == 0.0 {
                        DOP_MAX_FACTOR
                    } else {
                        DOP_MAX_FACTOR.min(DOP_SAFETY * error_norm.powf(DOP_ERR_EXPONENT))
                    };
                    // After a rejection, never grow the step on the retry.
                    let factor = if step_rejected { raw.min(1.0) } else { raw };
                    h_abs *= factor;

                    // Accept: advance, and compute the derivative at the new point
                    // for the next sub-step's first stage (DOP853's FSAL slot).
                    let t_start = t;
                    let y_start = y;
                    t = t_new;
                    y = y_new;
                    let (nfr, nfv) = derivative(force, epoch.shifted_by_seconds(t), &y)?;
                    fr = nfr;
                    fv = nfv;
                    on_accept(t_start, h, &y_start, &y, &kr, &kv, nfr, nfv)?;
                    break;
                }
                h_abs *= DOP_MIN_FACTOR.max(DOP_SAFETY * error_norm.powf(DOP_ERR_EXPONENT));
                step_rejected = true;
            }
        }
        Ok(y)
    }

    /// Advance `state` by `dt` **and** emit the dense-output segments spanning
    /// `[epoch, epoch + dt]` (§10.9). Returns the state at `epoch + dt` (identical
    /// to [`Integrator::step`]) together with one [`DenseSegment`] per accepted
    /// adaptive sub-step, in integration order — the continuous extension the
    /// [`clock`](crate::clock) samples for sub-snapshot queries.
    ///
    /// Each recorded step costs **3 force evaluations** beyond a plain step (the
    /// extra dense-output stages), so this is the deliberately-more-expensive path
    /// taken only when the interpolant is wanted. Backward spans (`dt < 0`) emit
    /// segments with `t1 < t0`; [`DenseSegment`] evaluates either direction.
    pub fn step_dense(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
    ) -> Result<(StateVector, Vec<DenseSegment>), IntegratorError> {
        let t0_abs = epoch.tdb_seconds_past_j2000();
        let mut segments: Vec<DenseSegment> = Vec::new();
        let end = self.integrate(
            force,
            epoch,
            state,
            dt,
            |t_off, h, y0, y1, kr, kv, fsal_r, fsal_v| {
                let seg = DenseSegment::build(
                    force,
                    epoch.shifted_by_seconds(t_off),
                    t0_abs + t_off,
                    h,
                    y0,
                    y1,
                    kr,
                    kv,
                    fsal_r,
                    fsal_v,
                )?;
                segments.push(seg);
                Ok(())
            },
        )?;
        Ok((end, segments))
    }
}

/// One accepted DOP853 sub-step's **dense output**: the degree-7 continuous
/// extension `y(t)` valid across `[t0, t1]` (§10.9).
///
/// Built by [`Dop853::step_dense`] from the 12 step stages, the FSAL derivative,
/// and 3 extra stage evaluations, this reproduces the integrator's own accuracy
/// *between* its steps — [`eval`](DenseSegment::eval) at either endpoint returns
/// the integrated state exactly, and interior points interpolate through the
/// trajectory's curvature (not linearly across it). Both position and velocity
/// come out of the same interpolant; per DOP853's construction the interpolated
/// velocity is not the exact time-derivative of the interpolated position (they
/// agree only to interpolation order) — this matches Hairer/SciPy and is correct.
///
/// `t0`/`t1` are absolute TDB seconds past J2000; `t1 - t0 = h` is signed (a
/// backward step has `t1 < t0`). [`lo`](DenseSegment::lo)/[`hi`](DenseSegment::hi)
/// give the covered interval regardless of direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DenseSegment {
    /// Segment start, absolute TDB seconds past J2000.
    t0: f64,
    /// Signed step length `t1 - t0`, seconds.
    h: f64,
    /// State at `t0` (the interpolation base point).
    y0: StateVector,
    /// Interpolation coefficients for position (`F[0..7]`, SciPy convention).
    fr: [nalgebra::Vector3<f64>; 7],
    /// Interpolation coefficients for velocity (`F[0..7]`, SciPy convention).
    fv: [nalgebra::Vector3<f64>; 7],
}

impl DenseSegment {
    /// Assemble the interpolant for one accepted sub-step. Computes the 3 extra
    /// dense-output stages (the only new force evaluations) from the already-known
    /// 12 step stages `(kr, kv)` and the FSAL derivative `(fsal_r, fsal_v)`, then
    /// forms the 7 interpolation coefficients per SciPy's `_dense_output_impl`:
    /// `F[0] = Δy`, `F[1] = h·f₀ − Δy`, `F[2] = 2Δy − h·(f₁ + f₀)`, and
    /// `F[3..7] = h · D · K` over the full 16-stage `K`.
    #[allow(clippy::too_many_arguments)]
    fn build(
        force: &dyn ForceModel,
        step_epoch: Epoch,
        t0: f64,
        h: f64,
        y0: &StateVector,
        y1: &StateVector,
        kr: &StageDerivs,
        kv: &StageDerivs,
        fsal_r: nalgebra::Vector3<f64>,
        fsal_v: nalgebra::Vector3<f64>,
    ) -> Result<Self, IntegratorError> {
        use dop853_tableau::{A_EXTRA, C_EXTRA, D};
        use nalgebra::Vector3;

        // Full 16-stage K: the 12 step stages, the FSAL derivative (K[12]), then
        // the 3 extra stages (K[13..16]) evaluated at C_EXTRA·h and coupled by
        // A_EXTRA against all earlier stages.
        let mut kr16 = [Vector3::zeros(); 16];
        let mut kv16 = [Vector3::zeros(); 16];
        kr16[..12].copy_from_slice(kr);
        kv16[..12].copy_from_slice(kv);
        kr16[12] = fsal_r;
        kv16[12] = fsal_v;

        for (e, a_row) in A_EXTRA.iter().enumerate() {
            let s = 13 + e;
            let mut dr = Vector3::zeros();
            let mut dv = Vector3::zeros();
            for j in 0..s {
                let a = a_row[j];
                if a != 0.0 {
                    dr += a * kr16[j];
                    dv += a * kv16[j];
                }
            }
            let stage = StateVector::new(y0.position + h * dr, y0.velocity + h * dv);
            let acc = force.acceleration(step_epoch.shifted_by_seconds(C_EXTRA[e] * h), &stage)?;
            kr16[s] = stage.velocity;
            kv16[s] = acc;
        }

        let delta_r = y1.position - y0.position;
        let delta_v = y1.velocity - y0.velocity;

        let mut fr = [Vector3::zeros(); 7];
        let mut fv = [Vector3::zeros(); 7];
        // F[0] = Δy
        fr[0] = delta_r;
        fv[0] = delta_v;
        // F[1] = h·f₀ − Δy  (f₀ = K[0])
        fr[1] = h * kr16[0] - delta_r;
        fv[1] = h * kv16[0] - delta_v;
        // F[2] = 2Δy − h·(f₁ + f₀)  (f₁ = FSAL = K[12])
        fr[2] = 2.0 * delta_r - h * (kr16[12] + kr16[0]);
        fv[2] = 2.0 * delta_v - h * (kv16[12] + kv16[0]);
        // F[3..7] = h · D · K
        for (k, d_row) in D.iter().enumerate() {
            let mut sr = Vector3::zeros();
            let mut sv = Vector3::zeros();
            for s in 0..16 {
                let d = d_row[s];
                if d != 0.0 {
                    sr += d * kr16[s];
                    sv += d * kv16[s];
                }
            }
            fr[3 + k] = h * sr;
            fv[3 + k] = h * sv;
        }

        Ok(Self {
            t0,
            h,
            y0: *y0,
            fr,
            fv,
        })
    }

    /// Lower bound of the covered interval, absolute TDB seconds (`min(t0, t1)`).
    pub fn lo(&self) -> f64 {
        self.t0.min(self.t0 + self.h)
    }

    /// Upper bound of the covered interval, absolute TDB seconds (`max(t0, t1)`).
    pub fn hi(&self) -> f64 {
        self.t0.max(self.t0 + self.h)
    }

    /// Whether `t` (absolute TDB seconds) falls within this segment's covered
    /// interval, up to a small slack for the shared endpoints between segments.
    pub fn contains(&self, t: f64) -> bool {
        let slack = 1e-6 * self.h.abs().max(1.0);
        t >= self.lo() - slack && t <= self.hi() + slack
    }

    /// Evaluate the interpolant at absolute TDB second `t`. For `t` inside
    /// `[lo, hi]` this is the 7th-order dense output; the two endpoints return the
    /// integrated states exactly. Evaluating outside the interval extrapolates the
    /// polynomial (the clock never does this — it selects the covering segment).
    ///
    /// Reversed-Horner form (SciPy `Dop853DenseOutput._call_impl`): with the
    /// normalized coordinate `x = (t − t0)/h`, accumulate the coefficients from
    /// `F[6]` down to `F[0]`, multiplying by `x` and `(1 − x)` on alternate terms,
    /// then add the base state `y0`.
    pub fn eval(&self, t: f64) -> StateVector {
        let x = (t - self.t0) / self.h;
        let mut yr = nalgebra::Vector3::zeros();
        let mut yv = nalgebra::Vector3::zeros();
        for i in 0..7 {
            let idx = 6 - i;
            yr += self.fr[idx];
            yv += self.fv[idx];
            if i % 2 == 0 {
                yr *= x;
                yv *= x;
            } else {
                yr *= 1.0 - x;
                yv *= 1.0 - x;
            }
        }
        StateVector::new(self.y0.position + yr, self.y0.velocity + yv)
    }
}

impl Integrator for Dop853 {
    fn step(
        &self,
        force: &dyn ForceModel,
        epoch: Epoch,
        state: &StateVector,
        dt: f64,
    ) -> Result<StateVector, IntegratorError> {
        // Plain step: the shared adaptive loop with a no-op accept hook (no dense
        // segments recorded, so no extra force evaluations).
        self.integrate(force, epoch, state, dt, |_, _, _, _, _, _, _, _| Ok(()))
    }
}

/// DOP853 Butcher tableau + embedded-error weights + dense-output tables,
/// transcribed verbatim from SciPy's `scipy/integrate/_ivp/dop853_coefficients.py`
/// (v1.17.1), itself a copy of Hairer, Nørsett & Wanner's published constants. The
/// step tableau (`A`, `B`, `C`) keeps the 12 stages of the step; the dense-output
/// tables (`C_EXTRA`, `A_EXTRA`, `D`, §10.9) add the 3 extra stages the continuous
/// extension needs on top of the FSAL derivative. `E5`/`E3` carry a trailing zero
/// for the FSAL row so the index range matches SciPy's `K`, even though that stage
/// contributes nothing to the error estimate.
mod dop853_tableau {
    /// Stage-coupling coefficients (strictly lower-triangular).
    pub const A: [[f64; 12]; 12] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [
            0.05260015195876773,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.0197250569845379,
            0.0591751709536137,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.02958758547680685,
            0.0,
            0.08876275643042054,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.2413651341592667,
            0.0,
            -0.8845494793282861,
            0.924834003261792,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.037037037037037035,
            0.0,
            0.0,
            0.17082860872947386,
            0.12546768756682242,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.037109375,
            0.0,
            0.0,
            0.17025221101954405,
            0.06021653898045596,
            -0.017578125,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.03709200011850479,
            0.0,
            0.0,
            0.17038392571223998,
            0.10726203044637328,
            -0.015319437748624402,
            0.008273789163814023,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.6241109587160757,
            0.0,
            0.0,
            -3.3608926294469414,
            -0.868219346841726,
            27.59209969944671,
            20.154067550477894,
            -43.48988418106996,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.47766253643826434,
            0.0,
            0.0,
            -2.4881146199716677,
            -0.590290826836843,
            21.230051448181193,
            15.279233632882423,
            -33.28821096898486,
            -0.020331201708508627,
            0.0,
            0.0,
            0.0,
        ],
        [
            -0.9371424300859873,
            0.0,
            0.0,
            5.186372428844064,
            1.0914373489967295,
            -8.149787010746927,
            -18.52006565999696,
            22.739487099350505,
            2.4936055526796523,
            -3.0467644718982196,
            0.0,
            0.0,
        ],
        [
            2.273310147516538,
            0.0,
            0.0,
            -10.53449546673725,
            -2.0008720582248625,
            -17.9589318631188,
            27.94888452941996,
            -2.8589982771350235,
            -8.87285693353063,
            12.360567175794303,
            0.6433927460157636,
            0.0,
        ],
    ];

    /// Weights for the 8th-order solution.
    pub const B: [f64; 12] = [
        0.054293734116568765,
        0.0,
        0.0,
        0.0,
        0.0,
        4.450312892752409,
        1.8915178993145003,
        -5.801203960010585,
        0.3111643669578199,
        -0.1521609496625161,
        0.20136540080403034,
        0.04471061572777259,
    ];

    /// Stage nodes (fraction of the step at which each stage is evaluated).
    pub const C: [f64; 12] = [
        0.0,
        0.05260015195876773,
        0.0789002279381516,
        0.1183503419072274,
        0.2816496580927726,
        0.3333333333333333,
        0.25,
        0.3076923076923077,
        0.6512820512820513,
        0.6,
        0.8571428571428571,
        1.0,
    ];

    /// 5th-order embedded-error weights (trailing FSAL entry is zero).
    pub const E5: [f64; 13] = [
        0.01312004499419488,
        0.0,
        0.0,
        0.0,
        0.0,
        -1.2251564463762044,
        -0.4957589496572502,
        1.6643771824549864,
        -0.35032884874997366,
        0.3341791187130175,
        0.08192320648511571,
        -0.022355307863886294,
        0.0,
    ];

    /// 3rd-order embedded-error weights (trailing FSAL entry is zero).
    pub const E3: [f64; 13] = [
        -0.18980075407240762,
        0.0,
        0.0,
        0.0,
        0.0,
        4.450312892752409,
        1.8915178993145003,
        -5.801203960010585,
        -0.4226823213237919,
        -0.1521609496625161,
        0.20136540080403034,
        0.02265179219836082,
        0.0,
    ];

    // ---- Dense output (7th-order continuous extension, §10.9) --------------
    //
    // DOP853's dense output builds a degree-7 interpolant per accepted step from
    // the 12 step stages, the FSAL derivative (K[12]), and **3 extra** stage
    // evaluations (K[13], K[14], K[15]) at the nodes in [`C_EXTRA`], coupled by
    // the rows in [`A_EXTRA`]. The interpolation coefficients `F[3..7]` are then
    // `h · D · K` over the full 16-stage `K` (see [`super::DenseSegment`]). All
    // three tables are transcribed from the same SciPy v1.17.1
    // `dop853_coefficients.py` as the step tableau above (there `A_EXTRA =
    // A[13:]`, `C_EXTRA = C[13:]`, `D` is `(INTERPOLATOR_POWER-3, N_STAGES_EXTENDED)
    // = (4, 16)`). Unlike the step tableau, these have **no** cheap self-consistency
    // identity — they are pinned instead by the interior-point polynomial
    // reproduction tests (a mistyped `D` still matches both step endpoints but
    // breaks the interior; see the dense-output tests).

    /// Nodes of the 3 extra dense-output stages (K[13], K[14], K[15]), as
    /// fractions of the step. (`C[13:16]` in SciPy.)
    pub const C_EXTRA: [f64; 3] = [0.1, 0.2, 0.7777777777777778];

    /// Stage-coupling rows for the 3 extra dense-output stages. Row `e` (for
    /// stage `13 + e`) dots against the already-computed `K[0..13+e]` (all step
    /// stages plus the FSAL derivative and any earlier extra stages), so the
    /// arrays are length 16 with trailing zeros. (`A[13:16]` in SciPy.)
    pub const A_EXTRA: [[f64; 16]; 3] = [
        [
            0.056167502283047954,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25350021021662483,
            -0.2462390374708025,
            -0.12419142326381637,
            0.15329179827876568,
            0.00820105229563469,
            0.007567897660545699,
            -0.008298,
            0.0,
            0.0,
            0.0,
        ],
        [
            0.03183464816350214,
            0.0,
            0.0,
            0.0,
            0.0,
            0.028300909672366776,
            0.053541988307438566,
            -0.05492374857139099,
            0.0,
            0.0,
            -0.00010834732869724932,
            0.0003825710908356584,
            -0.00034046500868740456,
            0.1413124436746325,
            0.0,
            0.0,
        ],
        [
            -0.42889630158379194,
            0.0,
            0.0,
            0.0,
            0.0,
            -4.697621415361164,
            7.683421196062599,
            4.06898981839711,
            0.3567271874552811,
            0.0,
            0.0,
            0.0,
            -0.0013990241651590145,
            2.9475147891527724,
            -9.15095847217987,
            0.0,
        ],
    ];

    /// Interpolation-coefficient weights for `F[3], F[4], F[5], F[6]`: each row
    /// `k` gives `F[3+k] = h · Σₛ D[k][s]·K[s]` over the full 16-stage `K`.
    /// (`D`, shape `(4, 16)`, in SciPy.)
    pub const D: [[f64; 16]; 4] = [
        [
            -8.428938276109013,
            0.0,
            0.0,
            0.0,
            0.0,
            0.5667149535193777,
            -3.0689499459498917,
            2.38466765651207,
            2.117034582445028,
            -0.871391583777973,
            2.2404374302607883,
            0.6315787787694688,
            -0.08899033645133331,
            18.148505520854727,
            -9.194632392478356,
            -4.436036387594894,
        ],
        [
            10.427508642579134,
            0.0,
            0.0,
            0.0,
            0.0,
            242.28349177525817,
            165.20045171727028,
            -374.5467547226902,
            -22.113666853125306,
            7.733432668472264,
            -30.674084731089398,
            -9.332130526430229,
            15.697238121770845,
            -31.139403219565178,
            -9.35292435884448,
            35.81684148639408,
        ],
        [
            19.985053242002433,
            0.0,
            0.0,
            0.0,
            0.0,
            -387.0373087493518,
            -189.17813819516758,
            527.8081592054236,
            -11.57390253995963,
            6.8812326946963,
            -1.0006050966910838,
            0.7777137798053443,
            -2.778205752353508,
            -60.19669523126412,
            84.32040550667716,
            11.99229113618279,
        ],
        [
            -25.69393346270375,
            0.0,
            0.0,
            0.0,
            0.0,
            -154.18974869023643,
            -231.5293791760455,
            357.6391179106141,
            93.40532418362432,
            -37.45832313645163,
            104.0996495089623,
            29.8402934266605,
            -43.53345659001114,
            96.32455395918828,
            -39.17726167561544,
            -149.72683625798564,
        ],
    ];
}

/// The first-order derivative `ẏ = (ṙ, a)` of the equation of motion at
/// `(epoch, state)`: position-rate is the velocity, velocity-rate is the force
/// model's acceleration. Shared by [`Dop853`]'s stages and initial-step routine.
fn derivative(
    force: &dyn ForceModel,
    epoch: Epoch,
    state: &StateVector,
) -> Result<(nalgebra::Vector3<f64>, nalgebra::Vector3<f64>), IntegratorError> {
    let acc = force.acceleration(epoch, state)?;
    Ok((state.velocity, acc))
}

/// Root-mean-square of a `(r, v)` 6-vector scaled component-wise by `(sr, sv)` —
/// the `‖x/scale‖ / √n` norm SciPy's initial-step routine uses (`n = 6`).
fn rms_scaled(
    ar: nalgebra::Vector3<f64>,
    av: nalgebra::Vector3<f64>,
    sr: nalgebra::Vector3<f64>,
    sv: nalgebra::Vector3<f64>,
) -> f64 {
    let x = ar.component_div(&sr);
    let y = av.component_div(&sv);
    ((x.norm_squared() + y.norm_squared()) / 6.0).sqrt()
}

/// Advance `state0` (at `epoch0`) by `total_dt` seconds in `n_steps` equal
/// fixed steps of `total_dt / n_steps`, returning the final state.
///
/// A convenience over [`Integrator::step`] for fixed-cadence propagation (and the
/// convergence-order tests, which compare the same arc at `N` vs `2N` steps).
/// `n_steps` must be at least 1.
pub fn propagate_fixed(
    integrator: &dyn Integrator,
    force: &dyn ForceModel,
    epoch0: Epoch,
    state0: StateVector,
    total_dt: f64,
    n_steps: u32,
) -> Result<StateVector, IntegratorError> {
    assert!(n_steps >= 1, "propagate_fixed needs at least one step");
    let h = total_dt / (n_steps as f64);
    let mut state = state0;
    let mut epoch = epoch0;
    for _ in 0..n_steps {
        state = integrator.step(force, epoch, &state, h)?;
        epoch = epoch.shifted_by_seconds(h);
    }
    Ok(state)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use nalgebra::Vector3;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    /// A spatially-uniform, constant-in-time acceleration `g` (a test-only force).
    /// RK4 integrates `r̈ = g` exactly, so it pins the stage-weighting arithmetic.
    struct UniformField {
        g: Vector3<f64>,
    }
    impl ForceModel for UniformField {
        fn acceleration(
            &self,
            _epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(self.g)
        }
    }

    /// A spatially-uniform acceleration that is **linear in time**: `a(t) = c·t`
    /// with `t` measured in seconds past J2000. RK4 integrates a system whose
    /// acceleration is a cubic-or-lower polynomial in `t` *exactly*, so a nonzero
    /// residual here can only come from evaluating a stage at the wrong epoch —
    /// this is the cheap, self-contained epoch-threading probe.
    struct LinearInTimeField {
        c: Vector3<f64>,
    }
    impl ForceModel for LinearInTimeField {
        fn acceleration(
            &self,
            epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(self.c * epoch.tdb_seconds_past_j2000())
        }
    }

    #[test]
    fn constant_acceleration_is_integrated_exactly() {
        let g = Vector3::new(0.3, -1.2, 0.7);
        let field = UniformField { g };
        let r0 = Vector3::new(1.0, 2.0, -3.0);
        let v0 = Vector3::new(-0.5, 0.4, 0.9);
        let s0 = StateVector::new(r0, v0);
        let t = 10.0;

        let end = propagate_fixed(&Rk4, &field, epoch0(), s0, t, 7).unwrap();
        // Closed form: r = r0 + v0 t + ½ g t², v = v0 + g t.
        let r_exact = r0 + v0 * t + 0.5 * g * t * t;
        let v_exact = v0 + g * t;
        assert!(
            (end.position - r_exact).norm() < 1e-9,
            "pos {:?}",
            end.position
        );
        assert!(
            (end.velocity - v_exact).norm() < 1e-12,
            "vel {:?}",
            end.velocity
        );
    }

    #[test]
    fn linear_in_time_acceleration_pins_epoch_threading() {
        // a(t) = c t  ⇒  v(t) = v0 + ½ c t²,  r(t) = r0 + v0 t + (1/6) c t³
        // (integrating from t=0). RK4 is exact for this cubic-in-t trajectory, so
        // any epoch-threading bug (all stages at t) breaks the match immediately.
        let c = Vector3::new(2.0, -1.0, 0.5);
        let field = LinearInTimeField { c };
        let r0 = Vector3::new(0.0, 0.0, 0.0);
        let v0 = Vector3::new(1.0, 2.0, 3.0);
        let s0 = StateVector::new(r0, v0);
        let t = 6.0;

        let end = propagate_fixed(&Rk4, &field, epoch0(), s0, t, 5).unwrap();
        let r_exact = r0 + v0 * t + c * (t * t * t) / 6.0;
        let v_exact = v0 + 0.5 * c * t * t;
        assert!(
            (end.position - r_exact).norm() < 1e-6,
            "pos {:?} vs {:?}",
            end.position,
            r_exact
        );
        assert!(
            (end.velocity - v_exact).norm() < 1e-9,
            "vel {:?} vs {:?}",
            end.velocity,
            v_exact
        );
    }

    #[test]
    fn a_single_step_forward_then_back_returns_to_start() {
        // Reversibility of the stepper itself on a two-body field: step +h, then
        // step −h from there, recover the seed (RK4 is not time-symmetric, so this
        // holds to truncation order, not exactly — hence the modest tolerance).
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let h = 3600.0; // one hour

        let fwd = Rk4.step(&field, epoch0(), &s0, h).unwrap();
        let back = Rk4
            .step(&field, epoch0().shifted_by_seconds(h), &fwd, -h)
            .unwrap();
        assert!(
            (back.position - s0.position).norm() / au < 1e-10,
            "pos err {:.3e}",
            (back.position - s0.position).norm() / au
        );
    }

    #[test]
    fn object_safe_as_dyn_integrator() {
        let field = PointMassGravity::new(vec![(1.0, FixedPerturber::at_origin()).into()]);
        let dynamic: &dyn Integrator = &Rk4;
        let s = StateVector::from_components(1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(dynamic.step(&field, epoch0(), &s, 0.01).is_ok());
    }

    // ---- DOP853 -----------------------------------------------------------

    /// The transcribed Butcher tableau must satisfy the identities every RK pair
    /// obeys, independent of *which* numbers were copied: each stage's coupling
    /// row sums to its node (`Σⱼ A[i][j] = C[i]`, autonomous consistency), the
    /// solution weights sum to one (`Σ B = 1`), and each embedded-error weight
    /// vector — a difference of two consistent methods — sums to zero. A single
    /// mistyped constant breaks one of these far outside round-off, so this is the
    /// cheap guard on the transcription that the whole integrator rests on.
    #[test]
    fn dop853_tableau_is_internally_consistent() {
        use super::dop853_tableau::{A, B, C, E3, E5};
        for i in 0..12 {
            let rowsum: f64 = A[i].iter().sum();
            assert!(
                (rowsum - C[i]).abs() < 1e-14,
                "row {i}: Σ A = {rowsum}, C = {}",
                C[i]
            );
        }
        assert!((B.iter().sum::<f64>() - 1.0).abs() < 1e-14, "Σ B ≠ 1");
        assert!(E5.iter().sum::<f64>().abs() < 1e-14, "Σ E5 ≠ 0");
        assert!(E3.iter().sum::<f64>().abs() < 1e-14, "Σ E3 ≠ 0");
    }

    #[test]
    fn dop853_integrates_constant_acceleration_exactly() {
        // DOP853 is exact for polynomial-in-t trajectories up to order 8, so a
        // constant field (quadratic trajectory) is reproduced to round-off.
        let g = Vector3::new(0.3, -1.2, 0.7);
        let field = UniformField { g };
        let r0 = Vector3::new(1.0, 2.0, -3.0);
        let v0 = Vector3::new(-0.5, 0.4, 0.9);
        let s0 = StateVector::new(r0, v0);
        let t = 10.0;

        let end = Dop853::new().step(&field, epoch0(), &s0, t).unwrap();
        let r_exact = r0 + v0 * t + 0.5 * g * t * t;
        let v_exact = v0 + g * t;
        assert!(
            (end.position - r_exact).norm() < 1e-6,
            "pos {:?}",
            end.position
        );
        assert!(
            (end.velocity - v_exact).norm() < 1e-9,
            "vel {:?}",
            end.velocity
        );
    }

    #[test]
    fn dop853_linear_in_time_field_pins_epoch_threading() {
        // a(t) = c·t ⇒ cubic-in-t trajectory, which DOP853 integrates exactly.
        // Evaluating any stage at the wrong epoch breaks the match immediately —
        // the adaptive analogue of the RK4 epoch-threading probe.
        //
        // Tolerance floor: hifitime quantizes epochs to integer nanoseconds, so a
        // stage at `epoch + C[s]·h` (DOP853's nodes are irrational fractions of h)
        // lands ~0.5 ns off its true sub-step time. A field that reads *absolute*
        // time turns that into an ~|c|·1e-9 acceleration error, accumulating to
        // ~1e-8 in velocity — a hifitime resolution artifact, not an integrator
        // bug. The 1e-6 bound sits well above that floor yet ~7 orders below the
        // O(tens) error a genuinely broken epoch threading (all stages at `t`)
        // would produce, so it still pins the thing under test.
        let c = Vector3::new(2.0, -1.0, 0.5);
        let field = LinearInTimeField { c };
        let r0 = Vector3::zeros();
        let v0 = Vector3::new(1.0, 2.0, 3.0);
        let s0 = StateVector::new(r0, v0);
        let t = 6.0;

        let end = Dop853::new().step(&field, epoch0(), &s0, t).unwrap();
        let r_exact = r0 + v0 * t + c * (t * t * t) / 6.0;
        let v_exact = v0 + 0.5 * c * t * t;
        assert!(
            (end.position - r_exact).norm() < 1e-6,
            "pos {:?}",
            end.position
        );
        assert!(
            (end.velocity - v_exact).norm() < 1e-6,
            "vel {:?}",
            end.velocity
        );
    }

    #[test]
    fn dop853_zero_dt_is_identity() {
        let field = PointMassGravity::new(vec![(1.0, FixedPerturber::at_origin()).into()]);
        let s = StateVector::from_components(1.0, 0.5, -0.2, 0.0, 1.0, 0.3);
        let end = Dop853::new().step(&field, epoch0(), &s, 0.0).unwrap();
        assert_eq!(end, s);
    }

    #[test]
    fn dop853_forward_then_back_recovers_the_seed() {
        // A full adaptive sweep out and back on a two-body field. Unlike the RK4
        // single-step probe, this crosses many sub-steps each way; agreement is
        // bounded by the accumulated local error (~tolerance), not machine ε.
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let span = 30.0 * 86_400.0; // 30 days

        let dop = Dop853::new().with_tolerances(1e-12, 1e-6);
        let fwd = dop.step(&field, epoch0(), &s0, span).unwrap();
        let back = dop
            .step(&field, epoch0().shifted_by_seconds(span), &fwd, -span)
            .unwrap();
        let rel = (back.position - s0.position).norm() / au;
        assert!(rel < 1e-9, "round-trip pos err {rel:.3e} (rel to 1 AU)");
    }

    #[test]
    fn dop853_max_substeps_fails_loud() {
        // A span that needs many sub-steps, with the budget pinned at 1, must
        // report MaxStepsExceeded rather than silently stopping short or spinning.
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let dop = Dop853::new().with_max_substeps(1);
        match dop.step(&field, epoch0(), &s0, 365.0 * 86_400.0) {
            Err(IntegratorError::MaxStepsExceeded { limit }) => assert_eq!(limit, 1),
            other => panic!("expected MaxStepsExceeded, got {other:?}"),
        }
    }

    #[test]
    fn dop853_object_safe_as_dyn_integrator() {
        let field = PointMassGravity::new(vec![(1.0, FixedPerturber::at_origin()).into()]);
        let dynamic: &dyn Integrator = &Dop853::new();
        let s = StateVector::from_components(1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(dynamic.step(&field, epoch0(), &s, 0.01).is_ok());
    }

    // ---- DOP853 dense output (§10.9) --------------------------------------

    /// A spatially-uniform acceleration that is a power of time: `a(t) = c·tᵖ`
    /// (t = seconds past J2000). The trajectory is a polynomial of degree `p + 2`,
    /// which DOP853's degree-7 dense output reproduces exactly for `p ≤ 5`.
    struct PowerInTimeField {
        c: Vector3<f64>,
        p: i32,
    }
    impl ForceModel for PowerInTimeField {
        fn acceleration(
            &self,
            epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(self.c * epoch.tdb_seconds_past_j2000().powi(self.p))
        }
    }

    /// A plain `step_dense` and `step` must integrate the *identical* trajectory —
    /// the dense path only records segments (extra stage evals that don't feed
    /// back), so the returned final states are bit-for-bit equal, and the dense
    /// eval reproduces the integrator's own state exactly at the step endpoints.
    #[test]
    fn dense_output_endpoints_match_the_step() {
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let span = 40.0 * 86_400.0;
        let dop = Dop853::new();

        let (end, segs) = dop.step_dense(&field, epoch0(), &s0, span).unwrap();
        let plain = dop.step(&field, epoch0(), &s0, span).unwrap();
        assert_eq!(
            end, plain,
            "dense and plain paths must integrate identically"
        );
        assert!(!segs.is_empty());

        let first = segs.first().unwrap();
        let last = segs.last().unwrap();
        // x=0 and x=1 zero out F[3..6], so endpoints recover the integrated states.
        assert!((first.eval(first.lo()).position - s0.position).norm() / au < 1e-12);
        assert!((last.eval(last.hi()).position - end.position).norm() / au < 1e-12);
        // Segments tile the span contiguously from the seed epoch to the target.
        assert!((first.lo() - epoch0().tdb_seconds_past_j2000()).abs() < 1e-6);
        assert!((last.hi() - span).abs() < 1e-6);
        for w in segs.windows(2) {
            assert!(
                (w[0].hi() - w[1].lo()).abs() < 1e-6,
                "segments must be contiguous"
            );
        }
    }

    /// The D-matrix pin. Endpoint continuity and the tableau consistency identities
    /// are both **blind to D** (F[3..6] vanish at x∈{0,1}), so D is only exercised
    /// at *interior* points. A polynomial trajectory of degree ≤ 7 is reproduced
    /// exactly by the correct dense output at every x; `p = 3` (a quintic path)
    /// makes the `h·D·K` coefficients genuinely nonzero, so a mistyped D — which
    /// still matches both endpoints — breaks the interior match here.
    #[test]
    fn dense_output_reproduces_polynomial_interior_pins_d() {
        for p in [1, 3] {
            let c = Vector3::new(0.7, -0.4, 0.2);
            let field = PowerInTimeField { c, p };
            let r0 = Vector3::new(1.0, -2.0, 0.5);
            let v0 = Vector3::new(0.3, 0.1, -0.2);
            let s0 = StateVector::new(r0, v0);
            let span = 8.0;

            let (_, segs) = Dop853::new()
                .step_dense(&field, epoch0(), &s0, span)
                .unwrap();
            assert!(!segs.is_empty());
            let (pp1, pp2) = ((p + 1) as f64, (p + 2) as f64);
            for seg in &segs {
                for f in [0.13, 0.37, 0.5, 0.71, 0.92] {
                    let t = seg.lo() + f * (seg.hi() - seg.lo());
                    let got = seg.eval(t);
                    let r_exact = r0 + v0 * t + c * t.powi(p + 2) / (pp1 * pp2);
                    let v_exact = v0 + c * t.powi(p + 1) / pp1;
                    // Relative bound: the interpolation error scales with the
                    // trajectory magnitude, while its floor is the hifitime ns
                    // quantization of the stage epochs (reading *absolute* tᵖ time
                    // amplifies a ~0.5 ns stage-epoch slip; see the sibling
                    // epoch-threading test). A mistyped D — invisible at the
                    // endpoints — instead injects an O(1)-relative spurious term at
                    // these interior points, so 1e-8 relative cleanly pins D.
                    let rel_pos = (got.position - r_exact).norm() / r_exact.norm().max(1.0);
                    let rel_vel = (got.velocity - v_exact).norm() / v_exact.norm().max(1.0);
                    assert!(
                        rel_pos < 1e-8,
                        "p={p}: interior pos rel err {rel_pos:.3e} at t={t}"
                    );
                    assert!(
                        rel_vel < 1e-8,
                        "p={p}: interior vel rel err {rel_vel:.3e} at t={t}"
                    );
                }
            }
        }
    }

    /// Independent (non-polynomial) confidence: a dense eval at an interior
    /// sub-step time equals a fresh integration to that same time, to ~integration
    /// tolerance. The two paths pick different internal steps, so agreement is to
    /// tolerance, not machine ε (HANDOFF §2 determinism note).
    #[test]
    fn dense_output_matches_reintegration_on_two_body() {
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let span = 60.0 * 86_400.0;
        let dop = Dop853::new();

        let (_, segs) = dop.step_dense(&field, epoch0(), &s0, span).unwrap();
        let seg = &segs[segs.len() / 2];
        let t = seg.lo() + 0.5 * (seg.hi() - seg.lo());
        let dense = seg.eval(t);

        let dt = t - epoch0().tdb_seconds_past_j2000();
        let reint = dop.step(&field, epoch0(), &s0, dt).unwrap();
        let rel = (dense.position - reint.position).norm() / au;
        assert!(rel < 1e-8, "dense vs reintegration rel err {rel:.3e}");
    }

    /// Dense output on a **backward** span (`dt < 0`): segments carry `t1 < t0`,
    /// and eval still reproduces the endpoints — the query convention is
    /// direction-agnostic via `lo`/`hi`.
    #[test]
    fn dense_output_backward_span_endpoints() {
        let mu = 1.327_124_400_18e20;
        let field = PointMassGravity::new(vec![(mu, FixedPerturber::at_origin()).into()]);
        let au = 1.495_978_707e11;
        let s0 = StateVector::from_components(au, 0.0, 0.0, 0.0, (mu / au).sqrt(), 0.0);
        let span = -20.0 * 86_400.0;
        let dop = Dop853::new();

        let (end, segs) = dop.step_dense(&field, epoch0(), &s0, span).unwrap();
        assert!(!segs.is_empty());
        // Integration ran backward: the first segment starts at the seed epoch...
        let first = segs.first().unwrap();
        let last = segs.last().unwrap();
        assert!((first.hi() - epoch0().tdb_seconds_past_j2000()).abs() < 1e-6);
        assert!((last.lo() - span).abs() < 1e-6);
        assert!((first.eval(first.hi()).position - s0.position).norm() / au < 1e-12);
        assert!((last.eval(last.lo()).position - end.position).norm() / au < 1e-12);
    }
}
