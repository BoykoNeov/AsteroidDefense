//! Tier 3: orbit uncertainty mapped from an initial-state covariance to the
//! b-plane, and from there to an impact probability (HANDOFF §5, §7, §10).
//!
//! Everything else in this crate answers a **deterministic** question: this rock,
//! this state, does it hit. Professional planetary defence does not reason that
//! way — it reasons over a covariance, because the orbit is *estimated* from a
//! finite observation arc and the honest answer is a probability. That is the
//! layer this module adds, and it is the layer keyholes only make sense inside.
//!
//! # The pipeline
//!
//! 1. A 6×6 [`StateCovariance`] on the asteroid's Cartesian state at the campaign
//!    start, barycentric ICRF, SI.
//! 2. A 2×6 Jacobian `J` of the b-plane coordinates with respect to that state,
//!    built by central differences over the real propagator ([`bplane_jacobian`]).
//! 3. The linear map `Σ_b = J Σ Jᵀ` — the 2×2 covariance of where the asteroid
//!    crosses the b-plane ([`BPlaneUncertainty`]).
//! 4. That Gaussian integrated over Earth's gravitationally-focused capture disc
//!    ([`BPlaneUncertainty::impact_probability`]).
//!
//! # Three decisions worth reading before using this
//!
//! **The reduction epoch is fixed, and it must be.** The obvious construction —
//! propagate each perturbed state, find *its own* closest approach, reduce that —
//! is wrong in a way that produces a plausible Jacobian. Closest approach is an
//! argmin over a sampled polyline; a small perturbation either moves the argmin by
//! a whole sample or does not move it at all, so the map from state to b-plane
//! coordinates is **quantised**, and its finite differences come back either noisy
//! or identically zero while the matrix still looks structurally fine. So every
//! run — nominal and perturbed alike — is reduced at **one fixed epoch** near the
//! nominal closest approach. This costs nothing in physics: the b-plane parameters
//! of the osculating geocentric hyperbola are asymptotic quantities, near-invariant
//! along the hyperbola, so sampling them at a fixed epoch rather than at perigee is
//! the same measurement taken somewhere better conditioned (see the numerical note
//! on [`BPlaneEncounter::from_relative_state`]).
//!
//! **The b-plane basis here is orthonormal but deliberately not *pinned*.** The
//! Öpik/Kizner ξ,ζ convention needs an external reference direction and is still
//! deferred (HANDOFF §Open questions). Nothing in this module needs it: under any
//! orthonormal change of b-plane basis — rotation *or* reflection — the mean
//! rotates, the covariance transforms as `RΣRᵀ`, and the capture disc is centred
//! at the origin and therefore invariant, so the probability integral is unchanged.
//! [`BPlaneBasis`] builds *an* arbitrary orthonormal frame perpendicular to `Ŝ`;
//! the tests pin that the probability does not care which one. Keyholes are what
//! will force the convention, because a resonant-return circle sits at a specific
//! ζ — and that is the next batch, not this one.
//!
//! **The sample cadence is a constant of this module, not a caller's choice.**
//! See [`SAMPLE_CADENCE_DAYS`].

use nalgebra::{Matrix2, Matrix2x6, Matrix6, Vector2, Vector3, Vector6};

use crate::geometry::BPlaneEncounter;
use crate::state::StateVector;

/// Snapshot cadence, in days, that every sample of the covariance mapping is
/// propagated at — **a constant of this module, deliberately not a parameter.**
///
/// `Clock::propagate` restarts the adaptive integrator at every snapshot, so the
/// cadence silently sets both the cost and the accuracy of a run: the shipping
/// 1-day cadence costs 9.4 s per 12-year re-fly, 10 days costs 1.1 s, and 30 days
/// costs 0.50 s. It is not free — the *absolute* b-plane perigee moves +118 m at
/// 10 days and +13.6 km at 30.
///
/// But a Jacobian column differences two runs flown at the same cadence, so the
/// systematic error is common to both and cancels. Measured (`probe_tier3_cost`)
/// on three columns — a coordinate velocity axis, the along-track velocity, and a
/// position component — `∂(perigee)/∂x` holds to **0.024 %** at 10 days and only
/// breaks (2.65 %) at 30. All three degrade by the same 2.65 %, so what the
/// cadence costs is a uniform scale factor, not anything direction-dependent.
///
/// Ten days is therefore the measured knee: an 8.7× speed-up for a quarter of a
/// per-mil on the only quantity this module consumes. It lives here rather than in
/// a caller's hands because **a Jacobian is only valid at the cadence its columns
/// converged at** — a frontend that dialled cadence for display reasons would
/// otherwise silently change the covariance answer while everything kept working.
/// [`cadence_is_pinned`](self) guards it: change this constant and re-measure, or
/// the test fails.
pub const SAMPLE_CADENCE_DAYS: f64 = 10.0;

/// Central-difference step for the three **position** columns, metres.
///
/// From the step-size study in `probe_tier3_cost`: at 10-day cadence the column
/// `∂(perigee)/∂r_x` is still truncation-dominated at 1e4 m (drifting 1.9 % per
/// halving), reaches a plateau at ~3.1e2 m (0.003 % per halving), and goes ragged
/// with round-off below ~1.6e2 m. This sits on the plateau.
///
/// It has no relationship to [`FD_STEP_VELOCITY_MS`] — metres and m/s share no
/// scale, and a step that suits one says nothing about the other. What they do
/// share is the *response* they provoke: both produce a b-plane excursion of order
/// 10–20 km, which is the real reason each is where it is.
pub const FD_STEP_POSITION_M: f64 = 312.5;

/// Central-difference step for the three **velocity** columns, m/s.
///
/// From the same study: `∂(perigee)/∂v_along` plateaus at 1.25e-4 m/s (0.007 % per
/// halving), is truncation-dominated above 5e-4, and goes ragged below 3e-5. See
/// [`FD_STEP_POSITION_M`] for why the two are chosen independently.
pub const FD_STEP_VELOCITY_MS: f64 = 1.25e-4;

/// Why an uncertainty computation could not be completed.
#[derive(Debug, Clone, PartialEq)]
pub enum UncertaintyError {
    /// The covariance matrix is not symmetric to within tolerance. Reported with
    /// the worst offending asymmetry so a caller can see whether it is a genuine
    /// error or a round-tripped matrix needing a symmetrising pass.
    NotSymmetric {
        /// Largest `|Σ_ij − Σ_ji|` found, in the matrix's own units.
        worst_asymmetry: f64,
    },
    /// The covariance matrix is not positive definite (Cholesky failed), so it is
    /// not a covariance: some direction in state space has zero or negative
    /// variance.
    NotPositiveDefinite,
    /// A matrix entry was not finite.
    NotFinite,
    /// A sample could not be flown or reduced — the perturbed trajectory left the
    /// scan gate, stopped being hyperbolic about Earth, or the propagation failed.
    /// Carries the underlying message; the column it belongs to is named too,
    /// because a Jacobian missing one column is not a Jacobian.
    SampleFailed {
        /// Which of the 6 state components was being perturbed (0–2 position,
        /// 3–5 velocity), or `None` for the nominal sample.
        column: Option<usize>,
        /// What went wrong underneath.
        message: String,
    },
    /// A non-positive or non-finite capture radius was handed to the probability
    /// integral.
    InvalidCaptureRadius(f64),
}

impl std::fmt::Display for UncertaintyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UncertaintyError::NotSymmetric { worst_asymmetry } => write!(
                f,
                "covariance is not symmetric (worst |Σij − Σji| = {worst_asymmetry:.6e})"
            ),
            UncertaintyError::NotPositiveDefinite => {
                write!(f, "covariance is not positive definite")
            }
            UncertaintyError::NotFinite => write!(f, "covariance contains a non-finite entry"),
            UncertaintyError::SampleFailed { column, message } => match column {
                Some(c) => write!(f, "covariance sample for column {c} failed: {message}"),
                None => write!(f, "nominal covariance sample failed: {message}"),
            },
            UncertaintyError::InvalidCaptureRadius(r) => {
                write!(
                    f,
                    "capture radius must be finite and positive (got {r:.6e})"
                )
            }
        }
    }
}

impl std::error::Error for UncertaintyError {}

/// A 6×6 covariance on a Cartesian state `(r, v)` — metres and m/s, barycentric
/// ICRF, ordered `[rx, ry, rz, vx, vy, vz]`.
///
/// Validated on construction: symmetric and positive definite, or it is not a
/// covariance and this refuses to build one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateCovariance {
    matrix: Matrix6<f64>,
}

impl StateCovariance {
    /// Relative tolerance for the symmetry check, against the matrix's own scale.
    const SYMMETRY_RTOL: f64 = 1.0e-10;

    /// Build from a 6×6 matrix, validating that it is finite, symmetric, and
    /// positive definite.
    ///
    /// Symmetry is checked *relatively* — against the largest absolute entry —
    /// because these matrices span wildly different units: a position variance is
    /// ~1e10 m² while a velocity variance is ~1e-8 m²/s², and an absolute
    /// tolerance that passes one rejects the other.
    pub fn new(matrix: Matrix6<f64>) -> Result<Self, UncertaintyError> {
        if matrix.iter().any(|x| !x.is_finite()) {
            return Err(UncertaintyError::NotFinite);
        }
        let scale = matrix.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        let mut worst = 0.0_f64;
        for i in 0..6 {
            for j in (i + 1)..6 {
                worst = worst.max((matrix[(i, j)] - matrix[(j, i)]).abs());
            }
        }
        if worst > Self::SYMMETRY_RTOL * scale.max(f64::MIN_POSITIVE) {
            return Err(UncertaintyError::NotSymmetric {
                worst_asymmetry: worst,
            });
        }
        if matrix.cholesky().is_none() {
            return Err(UncertaintyError::NotPositiveDefinite);
        }
        Ok(Self { matrix })
    }

    /// Build a diagonal covariance from six standard deviations, ordered
    /// `[σ_rx, σ_ry, σ_rz, σ_vx, σ_vy, σ_vz]` (m and m/s).
    ///
    /// A diagonal covariance in *inertial* axes is not what orbit determination
    /// produces — a real one is dominated by the along-track direction and is
    /// strongly correlated. Use this for tests and for building a covariance you
    /// then rotate; see [`Self::synthetic_along_track`] for the teaching case.
    pub fn from_sigmas(sigmas: [f64; 6]) -> Result<Self, UncertaintyError> {
        if sigmas.iter().any(|s| !s.is_finite() || *s <= 0.0) {
            return Err(UncertaintyError::NotPositiveDefinite);
        }
        let mut m = Matrix6::zeros();
        for (i, s) in sigmas.iter().enumerate() {
            m[(i, i)] = s * s;
        }
        Self::new(m)
    }

    /// **An invented covariance for a synthetic rock — not a measurement.**
    ///
    /// The campaign's threat is designed, not observed (HANDOFF §10): it has no
    /// observation arc, so it has no orbit-determination covariance, and there is
    /// no honest way to produce one. This builds a *plausible-shaped* one instead,
    /// and says so — the same rule every drawn body in this project follows, that a
    /// number either names its source or admits it has none. A real covariance
    /// arrives with a real asteroid, from the JPL Small-Body Database, in the
    /// keyhole batch.
    ///
    /// What is borrowed from reality is the **shape**, which is the part that
    /// matters for the lesson. NEO orbit uncertainty is overwhelmingly
    /// *along-track*: the sky-plane position is pinned well by astrometry while the
    /// position *along* the orbit is not, so the covariance ellipsoid is a long
    /// thin cigar lying down the velocity direction. That is why an impact
    /// prediction is a narrow ellipse on the b-plane rather than a circle, and why
    /// impact probability behaves so unlike intuition.
    ///
    /// Built in the seed's own along-track / radial / cross-track frame with
    /// `sigma_along_ms` down the velocity direction and `ratio` times less in the
    /// other two, then rotated into ICRF. Position uncertainty is implied
    /// dynamically rather than dialled independently: a velocity error *is* a
    /// growing position error, and inventing two independent numbers would invent
    /// a correlation structure this has no basis for. The position block is set
    /// isotropic and small, so the along-track velocity term dominates the map —
    /// which is the physical truth being taught.
    ///
    /// Returns `None` if the state is degenerate (zero velocity, or velocity
    /// parallel to position, so the frame cannot be built).
    pub fn synthetic_along_track(
        seed: StateVector,
        sigma_along_ms: f64,
        ratio: f64,
        sigma_position_m: f64,
    ) -> Option<Self> {
        if !(sigma_along_ms.is_finite()
            && sigma_along_ms > 0.0
            && ratio.is_finite()
            && ratio > 1.0
            && sigma_position_m.is_finite()
            && sigma_position_m > 0.0)
        {
            return None;
        }
        let v = seed.velocity;
        let vn = v.norm();
        if vn == 0.0 {
            return None;
        }
        let t_hat = v / vn; // along-track
        let h = seed.position.cross(&v);
        let hn = h.norm();
        if hn == 0.0 {
            return None;
        }
        let n_hat = h / hn; // orbit normal (cross-track)
        let r_hat = n_hat.cross(&t_hat); // completes the right-handed triad

        // Velocity block: σ_along down t̂, σ_along/ratio in the other two, then
        // rotated to ICRF as R diag(σ²) Rᵀ with R = [t̂ r̂ n̂] as columns.
        let s_a = sigma_along_ms;
        let s_o = sigma_along_ms / ratio;
        let vel_block = outer(t_hat, s_a * s_a) + outer(r_hat, s_o * s_o) + outer(n_hat, s_o * s_o);

        let mut m = Matrix6::zeros();
        for i in 0..3 {
            m[(i, i)] = sigma_position_m * sigma_position_m;
            for j in 0..3 {
                m[(3 + i, 3 + j)] = vel_block[(i, j)];
            }
        }
        Self::new(m).ok()
    }

    /// The underlying 6×6 matrix (m², m·m/s, m²/s² by block).
    pub fn matrix(&self) -> &Matrix6<f64> {
        &self.matrix
    }

    /// The `±n σ` shell along the covariance's own principal axes: 12 state
    /// offsets (6 eigenvectors × 2 signs), each scaled by `n·√λ`.
    ///
    /// This is the **linearity probe**, and it is deliberately deterministic rather
    /// than random. Whether the linear map is still a good description out at 3σ is
    /// not a question random sampling answers efficiently — a thousand isotropic
    /// draws mostly land near the middle where linearity was never in doubt. The
    /// extremes along the principal axes are exactly where it breaks first, and
    /// there are only twelve of them: if they map to an ellipse the linearisation
    /// holds, and if they map to a banana it does not.
    ///
    /// Returned as offsets to be *added* to the nominal seed, in the same
    /// `[r, v]` ordering.
    pub fn sigma_shell(&self, n_sigma: f64) -> Vec<Vector6<f64>> {
        let eig = self.matrix.symmetric_eigen();
        let mut out = Vec::with_capacity(12);
        for k in 0..6 {
            let lambda = eig.eigenvalues[k].max(0.0);
            let axis = eig.eigenvectors.column(k).into_owned();
            let scaled = axis * (n_sigma * lambda.sqrt());
            out.push(scaled);
            out.push(-scaled);
        }
        out
    }
}

/// `v vᵀ · s` — the rank-1 contribution of a unit axis with variance `s`.
fn outer(v: Vector3<f64>, s: f64) -> nalgebra::Matrix3<f64> {
    v * v.transpose() * s
}

/// An orthonormal 2-frame spanning the b-plane — the plane through Earth's centre
/// perpendicular to the incoming asymptote `Ŝ`.
///
/// **This is *an* orthonormal basis, not *the* Öpik/Kizner ξ,ζ frame.** That
/// convention needs an external reference direction and is still deferred
/// (HANDOFF §Open questions); nothing here needs it, because every quantity this
/// module reports is invariant under an orthonormal change of b-plane basis — the
/// capture disc is centred at the origin, so rotating or reflecting the frame
/// rotates the mean and the covariance together and leaves the probability alone.
/// The tests pin that invariance rather than asserting a convention, which is what
/// keeps settling it later free.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BPlaneBasis {
    /// Incoming-asymptote direction the plane is perpendicular to.
    pub s_hat: Vector3<f64>,
    /// First in-plane axis (unit, ⊥ `s_hat`).
    pub e1: Vector3<f64>,
    /// Second in-plane axis (unit, ⊥ `s_hat` and ⊥ `e1`).
    pub e2: Vector3<f64>,
}

impl BPlaneBasis {
    /// Build an arbitrary-but-deterministic orthonormal frame perpendicular to the
    /// encounter's `Ŝ`.
    ///
    /// The seed axis is whichever coordinate axis is *least* aligned with `Ŝ`,
    /// which keeps the cross product well conditioned for every `Ŝ` — the
    /// textbook "pick x unless you are pointing along x" has a degenerate band
    /// near the switch that this avoids.
    pub fn from_encounter(enc: &BPlaneEncounter) -> Self {
        let s = enc.s_hat.normalize();
        let seed = {
            let a = [s[0].abs(), s[1].abs(), s[2].abs()];
            let min_i = if a[0] <= a[1] && a[0] <= a[2] {
                0
            } else if a[1] <= a[2] {
                1
            } else {
                2
            };
            let mut e = Vector3::zeros();
            e[min_i] = 1.0;
            e
        };
        let e1 = s.cross(&seed).normalize();
        let e2 = s.cross(&e1);
        Self { s_hat: s, e1, e2 }
    }

    /// Project an encounter's b-vector onto this frame — the 2D b-plane
    /// coordinates, metres.
    ///
    /// Note the projection uses **this** basis, not one rebuilt from the encounter
    /// being projected. That is the point: a perturbed run's own asymptote differs
    /// slightly from the nominal's, and re-deriving the frame per sample would
    /// measure the frame's wobble alongside the physics. The nominal frame is held
    /// fixed and every sample is expressed in it, which is what makes the columns
    /// of the Jacobian commensurable.
    pub fn project(&self, enc: &BPlaneEncounter) -> Vector2<f64> {
        Vector2::new(enc.b_vector.dot(&self.e1), enc.b_vector.dot(&self.e2))
    }
}

/// Central-difference Jacobian `∂(b-plane coordinates)/∂(initial state)`, a 2×6.
///
/// `sample` maps a perturbed initial state to its b-plane coordinates — propagate,
/// reduce at the **fixed** epoch, project onto the **nominal** basis. It is a
/// closure rather than a concrete pipeline so this function stays pure and
/// kernel-free: the tests drive it with analytic maps whose Jacobians are known
/// exactly, which is the only way to tell a correct difference scheme from one
/// that merely produces plausible numbers.
///
/// Steps come from [`FD_STEP_POSITION_M`] and [`FD_STEP_VELOCITY_MS`] — different
/// per block, because metres and m/s share no scale.
///
/// Costs **12 samples**, one pair per column. At the module's cadence that is
/// about 13 seconds against the real propagator.
pub fn bplane_jacobian<F>(
    seed: StateVector,
    mut sample: F,
) -> Result<Matrix2x6<f64>, UncertaintyError>
where
    F: FnMut(StateVector) -> Result<Vector2<f64>, UncertaintyError>,
{
    let mut j = Matrix2x6::zeros();
    for col in 0..6 {
        let h = if col < 3 {
            FD_STEP_POSITION_M
        } else {
            FD_STEP_VELOCITY_MS
        };
        let plus = sample(offset(seed, col, h)).map_err(|e| tag(e, col))?;
        let minus = sample(offset(seed, col, -h)).map_err(|e| tag(e, col))?;
        let d = (plus - minus) / (2.0 * h);
        j[(0, col)] = d[0];
        j[(1, col)] = d[1];
    }
    Ok(j)
}

/// Add `h` to state component `col` (0–2 position, 3–5 velocity).
fn offset(mut s: StateVector, col: usize, h: f64) -> StateVector {
    if col < 3 {
        s.position[col] += h;
    } else {
        s.velocity[col - 3] += h;
    }
    s
}

/// Attach the column index to a sample failure, so a broken Jacobian says which
/// direction broke it rather than just that something did.
fn tag(e: UncertaintyError, col: usize) -> UncertaintyError {
    match e {
        UncertaintyError::SampleFailed { message, .. } => UncertaintyError::SampleFailed {
            column: Some(col),
            message,
        },
        other => other,
    }
}

/// Where the asteroid crosses the b-plane, as a Gaussian — and what fraction of
/// that Gaussian lands on Earth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BPlaneUncertainty {
    /// The nominal crossing point in the b-plane frame, metres.
    pub mean: Vector2<f64>,
    /// The 2×2 covariance of the crossing, m², in the same frame.
    pub covariance: Matrix2<f64>,
    /// The frame both of the above are expressed in.
    pub basis: BPlaneBasis,
    /// Earth's gravitationally-focused capture radius at this encounter, metres —
    /// the disc, centred on the b-plane origin, that counts as an impact.
    pub capture_radius: f64,
}

impl BPlaneUncertainty {
    /// Map a state covariance through a b-plane Jacobian: `Σ_b = J Σ Jᵀ`.
    pub fn from_jacobian(
        jacobian: &Matrix2x6<f64>,
        state_cov: &StateCovariance,
        nominal: &BPlaneEncounter,
        basis: BPlaneBasis,
    ) -> Self {
        let covariance = jacobian * state_cov.matrix() * jacobian.transpose();
        Self {
            mean: basis.project(nominal),
            covariance,
            basis,
            capture_radius: nominal.capture_radius,
        }
    }

    /// The 1σ semi-axes of the uncertainty ellipse, metres, largest first.
    ///
    /// Reported because the *shape* is the lesson: a real NEO covariance maps to
    /// something enormously elongated, and an operator who sees only a scalar
    /// probability never learns why moving along the long axis is nearly free
    /// while moving across it is not.
    pub fn sigma_axes(&self) -> (f64, f64) {
        let eig = self.covariance.symmetric_eigen();
        let mut l = [eig.eigenvalues[0].max(0.0), eig.eigenvalues[1].max(0.0)];
        if l[1] > l[0] {
            l.swap(0, 1);
        }
        (l[0].sqrt(), l[1].sqrt())
    }

    /// How many σ the nominal crossing sits from the b-plane **origin**, in the
    /// covariance's own metric (`√(μᵀ Σ⁻¹ μ)`, the Mahalanobis distance).
    ///
    /// **This is not "how many σ from a hit", and reading it that way inverts the
    /// answer.** The origin is Earth's *centre*; the thing that counts as an impact
    /// is a disc of [`capture_radius`] around it, which for a slow encounter is
    /// over 11 000 km wide. The shipping campaign's designed hit reports ~8 200 σ
    /// here — the ellipse is sub-kilometre across its minor axis, so the nominal
    /// really is thousands of ellipse-widths from dead centre — while its impact
    /// probability is exactly 1, because all of that sits deep inside the disc.
    ///
    /// What it is good for is comparing a *miss* against the spread that surrounds
    /// it: 50 000 km sounds safe and is safe against a 1 000 km ellipse, and is a
    /// coin toss against a 200 000 km one. The kilometre figure alone cannot
    /// distinguish those and this can — but the capture radius has to be in the
    /// comparison, which is why [`impact_probability`] is the number to quote and
    /// this one is the number that explains it.
    ///
    /// `None` if the covariance is singular.
    ///
    /// [`capture_radius`]: BPlaneUncertainty::capture_radius
    /// [`impact_probability`]: BPlaneUncertainty::impact_probability
    pub fn sigma_distance(&self) -> Option<f64> {
        let inv = self.covariance.try_inverse()?;
        let d2 = (self.mean.transpose() * inv * self.mean)[(0, 0)];
        if d2 < 0.0 {
            return None;
        }
        Some(d2.sqrt())
    }

    /// Impact probability: the b-plane Gaussian integrated over the capture disc.
    ///
    /// The domain is the disc of radius `capture_radius` centred on the b-plane
    /// **origin** (Earth), and the Gaussian is centred on the nominal crossing —
    /// so this is *not* a symmetric "how likely is the mean inside" question, it
    /// is the honest overlap of a predicted spread with a target.
    ///
    /// # Why this is not quadrature over the disc
    ///
    /// The obvious construction — polar coordinates on the capture disc, sample
    /// the Gaussian — fails exactly where this module operates. A well-determined
    /// orbit puts a 10 km ellipse inside an 11 311 km disc; radial nodes spread
    /// across the disc then sit tens of σ apart and step straight over the peak,
    /// returning 0.994 for an integral whose answer is 1. The mass is concentrated
    /// somewhere the domain's own parameterisation does not resolve.
    ///
    /// So the coordinates are chosen to suit the *integrand* instead. Whitening by
    /// the covariance's Cholesky factor (`u = L⁻¹(x − μ)`, `Σ = LLᵀ`) turns the
    /// Gaussian into a standard normal — unit scale, always resolved, at any
    /// elongation — and turns the disc into an ellipse. In polar coordinates on
    /// `u`, each direction `û(θ)` meets that ellipse in the interval between the
    /// roots of a quadratic, and the radial integral is then **analytic**:
    ///
    /// ```text
    ///   ∫ r·e^(−r²/2) dr = −e^(−r²/2)
    ///   P = (1/2π) ∮ [ e^(−r_min(θ)²/2) − e^(−r_max(θ)²/2) ] dθ
    /// ```
    ///
    /// What is left is one *periodic* 1-D integral, where equispaced sampling is
    /// spectrally accurate — refined by doubling until it stops moving, so an
    /// extremely elongated ellipse (where `r_min(θ)` turns sharp) is resolved
    /// rather than silently under-sampled at a fixed node count.
    ///
    /// The isotropic centred case reduces to `1 − exp(−R²/2σ²)` analytically here,
    /// and the tests check that closed form across four orders of magnitude of
    /// `R/σ` — a quadrature that cannot reproduce a known integral is not evidence
    /// of anything.
    pub fn impact_probability(&self) -> Result<f64, UncertaintyError> {
        if !(self.capture_radius.is_finite() && self.capture_radius > 0.0) {
            return Err(UncertaintyError::InvalidCaptureRadius(self.capture_radius));
        }
        let chol = self
            .covariance
            .cholesky()
            .ok_or(UncertaintyError::NotPositiveDefinite)?;
        let l = chol.l();
        let r2 = self.capture_radius * self.capture_radius;
        let mu2 = self.mean.norm_squared();

        // The θ-integrand: the standard-normal mass on the ray `û(θ)` that lands
        // inside the disc. `|L(r û) + μ|² ≤ R²` is a quadratic in `r`; the mass
        // between its roots is the difference of two Gaussian tails.
        let f = |theta: f64| {
            let u = Vector2::new(theta.cos(), theta.sin());
            let lu = l * u;
            let a = lu.norm_squared();
            if a <= 0.0 {
                return 0.0;
            }
            let b = 2.0 * lu.dot(&self.mean);
            let c = mu2 - r2;
            let disc = b * b - 4.0 * a * c;
            if disc <= 0.0 {
                // The ray never enters the disc.
                return 0.0;
            }
            let sq = disc.sqrt();
            let r_hi = (-b + sq) / (2.0 * a);
            if r_hi <= 0.0 {
                // The whole intersection lies behind the origin; the opposite
                // direction's own θ covers it.
                return 0.0;
            }
            let r_lo = ((-b - sq) / (2.0 * a)).max(0.0);
            (-0.5 * r_lo * r_lo).exp() - (-0.5 * r_hi * r_hi).exp()
        };

        // Equispaced average on [0, 2π), doubled until it converges. Nesting keeps
        // the refinement cheap: the doubled rule is the old average and the new
        // midpoints, averaged.
        const N_START: usize = 256;
        const N_MAX: usize = 262_144;
        const RTOL: f64 = 1.0e-13;
        let tau = std::f64::consts::TAU;
        let mut n = N_START;
        let mut mean_val = (0..n).map(|k| f(tau * k as f64 / n as f64)).sum::<f64>() / n as f64;
        while n < N_MAX {
            let mid = (0..n)
                .map(|k| f(tau * (k as f64 + 0.5) / n as f64))
                .sum::<f64>()
                / n as f64;
            let next = 0.5 * (mean_val + mid);
            let moved = (next - mean_val).abs();
            mean_val = next;
            n *= 2;
            if moved <= RTOL * mean_val.abs().max(1.0e-300) {
                break;
            }
        }
        Ok(mean_val.clamp(0.0, 1.0))
    }
}

/// The expensive half of the Tier-3 mapping — and the half that does **not**
/// depend on the covariance.
///
/// `Σ_b = J Σ Jᵀ` splits cleanly: `J` costs 13 propagations and describes the
/// *trajectory's* sensitivity, while `Σ` is a statement about how well the orbit
/// is known and costs nothing. Keeping them separate is what makes "the same rock
/// with a better-observed orbit" a free question instead of a 14-second one —
/// which matters, because that comparison is the entire Tier-3 lesson.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BPlaneSensitivity {
    /// The nominal encounter, reduced at the fixed epoch the columns were built
    /// around.
    pub nominal: BPlaneEncounter,
    /// The frame the Jacobian's rows are expressed in.
    pub basis: BPlaneBasis,
    /// `∂(b-plane coordinates)/∂(initial state)`, metres per metre and per m/s.
    pub jacobian: Matrix2x6<f64>,
}

impl BPlaneSensitivity {
    /// Push a covariance through: `Σ_b = J Σ Jᵀ`. Free.
    pub fn map(&self, covariance: &StateCovariance) -> BPlaneUncertainty {
        BPlaneUncertainty::from_jacobian(&self.jacobian, covariance, &self.nominal, self.basis)
    }

    /// The nominal crossing point in the b-plane frame, metres.
    pub fn mean(&self) -> Vector2<f64> {
        self.basis.project(&self.nominal)
    }
}

/// One `±nσ` shell sample: what the linear map predicted, and where the real
/// propagator actually put it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShellSample {
    /// The state offset from the nominal seed that was flown.
    pub offset: Vector6<f64>,
    /// `J · offset` — the linear prediction, metres in the b-plane frame.
    pub predicted: Vector2<f64>,
    /// The flown result, relative to the nominal crossing, same frame and units.
    pub flown: Vector2<f64>,
}

impl ShellSample {
    /// How far the prediction missed, in b-plane metres.
    ///
    /// Absolute, not relative — deliberately. Normalising each sample by *its own*
    /// flown displacement looks natural and is a trap: the shell includes the
    /// covariance's smallest principal axis, whose b-plane displacement can be a
    /// few metres, and dividing a metre-scale numerical residual by a metre-scale
    /// signal reports 100 % bending in a direction that contributes nothing to the
    /// ellipse. [`LinearityReport`] normalises against the shell's *largest*
    /// displacement instead, which is the scale the covariance actually has.
    pub fn residual(&self) -> f64 {
        (self.predicted - self.flown).norm()
    }
}

/// Whether the linear map still describes the encounter out at the edge of the
/// covariance — the question a Jacobian cannot answer about itself.
///
/// `Σ_b = J Σ Jᵀ` is exact only if the state→b-plane map is linear across the
/// covariance's support. It is not; it is linearised. Out at 3σ the real map may
/// have bent, in which case the ellipse is a fiction and the honest picture is a
/// banana. This is the deterministic probe of that: fly the twelve principal-axis
/// extremes and compare each against what `J` predicted.
///
/// Deterministic on purpose. A thousand random draws answer the same question far
/// more slowly and worse, because most of them land near the middle where
/// linearity was never in doubt; the principal-axis extremes are where it breaks
/// first, and there are only twelve.
#[derive(Debug, Clone, PartialEq)]
pub struct LinearityReport {
    /// Every shell sample, in [`StateCovariance::sigma_shell`] order.
    pub samples: Vec<ShellSample>,
    /// How many σ out the shell was flown.
    pub n_sigma: f64,
    /// The largest `|predicted − flown|` across the shell, b-plane metres.
    pub max_residual: f64,
    /// The largest flown displacement across the shell, b-plane metres — the
    /// distribution's own scale at `n_sigma`, and what the residual is judged
    /// against.
    pub shell_scale: f64,
    /// `max_residual / shell_scale` — how much the map bent, as a fraction of how
    /// far the shell reaches. This, not a per-sample ratio, is the number that says
    /// whether the ellipse is honest.
    pub max_relative_residual: f64,
    /// Index into `samples` of the largest residual.
    pub worst_index: usize,
}

impl LinearityReport {
    /// Build from the flown shell. `flown[i]` is the b-plane displacement from the
    /// nominal crossing produced by `offsets[i]`.
    ///
    /// Panics if the two slices differ in length — they are two halves of one
    /// measurement and a caller that lost track of the pairing has a bug this
    /// should not paper over.
    pub fn new(
        jacobian: &Matrix2x6<f64>,
        offsets: &[Vector6<f64>],
        flown: &[Vector2<f64>],
        n_sigma: f64,
    ) -> Self {
        assert_eq!(
            offsets.len(),
            flown.len(),
            "shell offsets and flown results must pair up one to one"
        );
        let samples: Vec<ShellSample> = offsets
            .iter()
            .zip(flown.iter())
            .map(|(o, f)| ShellSample {
                offset: *o,
                predicted: jacobian * o,
                flown: *f,
            })
            .collect();
        let (worst_index, max_residual) = samples
            .iter()
            .enumerate()
            .map(|(i, s)| (i, s.residual()))
            .fold((0, 0.0_f64), |acc, x| if x.1 > acc.1 { x } else { acc });
        let shell_scale = samples
            .iter()
            .map(|s| s.flown.norm())
            .fold(0.0_f64, f64::max);
        let max_relative_residual = if shell_scale > 0.0 {
            max_residual / shell_scale
        } else {
            0.0
        };
        Self {
            samples,
            n_sigma,
            max_residual,
            shell_scale,
            max_relative_residual,
            worst_index,
        }
    }

    /// Whether the linearisation holds to `tolerance` (a fraction, e.g. `0.05` for
    /// 5 %) everywhere on the shell.
    ///
    /// There is no universally right threshold — it depends on what the number is
    /// for. A probability quoted to one significant figure tolerates far more
    /// bending than a keyhole intersection does. So this takes the tolerance rather
    /// than hiding one.
    pub fn holds_within(&self, tolerance: f64) -> bool {
        self.max_relative_residual <= tolerance
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Matrix3;

    fn approx(a: f64, b: f64, rtol: f64) -> bool {
        (a - b).abs() <= rtol * a.abs().max(b.abs()).max(f64::MIN_POSITIVE)
    }

    // --- the pinned cadence -------------------------------------------------

    /// The cadence is a measurement, so it cannot drift without one.
    ///
    /// `probe_tier3_cost` measured the derivative error at 10 days as 0.024 % and
    /// at 30 days as 2.65 %. A future edit that "just makes it faster" by moving to
    /// 30 has silently multiplied every reported probability's underlying
    /// sensitivity by 1.027 — a change nothing else in the suite would catch,
    /// because every answer stays plausible. Change the constant only alongside a
    /// re-run of the probe, and update this test with the new measurement.
    #[test]
    fn cadence_is_pinned() {
        assert_eq!(
            SAMPLE_CADENCE_DAYS, 10.0,
            "sample cadence changed without re-measuring the derivative convergence \
             (probe_tier3_cost): 10 d held the Jacobian to 0.024%, 30 d broke it to 2.65%"
        );
        assert_eq!(FD_STEP_POSITION_M, 312.5);
        assert_eq!(FD_STEP_VELOCITY_MS, 1.25e-4);
    }

    // --- StateCovariance validation ------------------------------------------

    #[test]
    fn rejects_asymmetric_and_indefinite_and_nonfinite() {
        let mut m = Matrix6::identity();
        m[(0, 3)] = 1.0; // no matching (3, 0)
        assert!(matches!(
            StateCovariance::new(m),
            Err(UncertaintyError::NotSymmetric { .. })
        ));

        let mut m = Matrix6::identity();
        m[(2, 2)] = -1.0;
        assert_eq!(
            StateCovariance::new(m),
            Err(UncertaintyError::NotPositiveDefinite)
        );

        let mut m = Matrix6::identity();
        m[(1, 1)] = f64::NAN;
        assert_eq!(StateCovariance::new(m), Err(UncertaintyError::NotFinite));
    }

    /// The symmetry check must be relative, not absolute — a real state covariance
    /// spans ~18 orders of magnitude between its position and velocity blocks, so
    /// an absolute tolerance either rejects valid matrices or accepts broken ones.
    #[test]
    fn symmetry_tolerance_survives_the_units_spread() {
        // Position variance ~1e10 m², velocity variance ~1e-8 m²/s².
        let cov = StateCovariance::from_sigmas([1e5, 1e5, 1e5, 1e-4, 1e-4, 1e-4])
            .expect("well-scaled diagonal covariance");
        // A velocity-block asymmetry far below the *matrix* scale is accepted...
        let mut m = *cov.matrix();
        m[(3, 4)] = 1.0e-20;
        m[(4, 3)] = 2.0e-20;
        assert!(StateCovariance::new(m).is_ok());
        // ...while one comparable to the matrix scale is not.
        let mut m = *cov.matrix();
        m[(0, 1)] = 1.0e6;
        assert!(matches!(
            StateCovariance::new(m),
            Err(UncertaintyError::NotSymmetric { .. })
        ));
    }

    #[test]
    fn sigma_shell_is_twelve_offsets_on_the_principal_axes() {
        let cov = StateCovariance::from_sigmas([100.0, 200.0, 300.0, 1e-3, 2e-3, 3e-3]).unwrap();
        let shell = cov.sigma_shell(3.0);
        assert_eq!(shell.len(), 12);
        // Diagonal covariance ⇒ principal axes are the coordinate axes, so the
        // shell entries are ±3σ on one component each.
        let expected = [100.0, 200.0, 300.0, 1e-3, 2e-3, 3e-3];
        for v in &shell {
            let nz: Vec<usize> = (0..6).filter(|i| v[*i].abs() > 1e-18).collect();
            assert_eq!(nz.len(), 1, "shell offset should lie on one principal axis");
            let i = nz[0];
            assert!(approx(v[i].abs(), 3.0 * expected[i], 1e-12));
        }
    }

    /// The invented covariance must actually have the shape it claims: elongated
    /// down the velocity direction by the requested ratio.
    #[test]
    fn synthetic_covariance_is_along_track_dominated() {
        let seed = StateVector::new(
            Vector3::new(1.4e11, 3.0e10, -1.0e9),
            Vector3::new(-6.0e3, 2.8e4, 4.0e2),
        );
        let cov = StateCovariance::synthetic_along_track(seed, 5.0e-5, 20.0, 1.0e3)
            .expect("non-degenerate seed");
        let t_hat = seed.velocity.normalize();
        let vel = cov.matrix().fixed_view::<3, 3>(3, 3).into_owned();
        // Variance along t̂ is σ_along²; across it is (σ_along/ratio)².
        let along = (t_hat.transpose() * vel * t_hat)[(0, 0)];
        assert!(approx(along.sqrt(), 5.0e-5, 1e-12));
        let across = seed.position.cross(&seed.velocity).normalize();
        let a2 = (across.transpose() * vel * across)[(0, 0)];
        assert!(approx(a2.sqrt(), 5.0e-5 / 20.0, 1e-12));
        // Degenerate seeds are refused rather than producing a silent basis.
        let dead = StateVector::new(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros());
        assert!(StateCovariance::synthetic_along_track(dead, 1e-5, 10.0, 1e3).is_none());
    }

    // --- the finite-difference scheme ----------------------------------------

    /// A central difference of an exactly-linear map must return that map's matrix
    /// to machine precision. This is the test that distinguishes a correct
    /// difference scheme — right steps, right signs, right column order — from one
    /// that merely produces plausible-looking numbers against the real propagator,
    /// where nothing is known in advance.
    #[test]
    fn jacobian_reproduces_an_exactly_linear_map() {
        // A deliberately asymmetric matrix so a transposed or mis-ordered column
        // cannot pass by coincidence, with the position/velocity magnitude split a
        // real one has.
        let mut a = Matrix2x6::zeros();
        for c in 0..6 {
            a[(0, c)] = (c as f64 + 1.0) * if c < 3 { 1.0 } else { 1.0e7 };
            a[(1, c)] = -(2.0 * c as f64 + 0.5) * if c < 3 { 1.0 } else { 1.0e7 };
        }
        let seed = StateVector::new(
            Vector3::new(1.0e11, -2.0e10, 3.0e9),
            Vector3::new(1.0e4, 2.0e4, -3.0e3),
        );
        let f = |s: StateVector| {
            let x = Vector6::new(
                s.position[0],
                s.position[1],
                s.position[2],
                s.velocity[0],
                s.velocity[1],
                s.velocity[2],
            );
            Ok(a * x)
        };
        let j = bplane_jacobian(seed, f).expect("linear map samples cleanly");
        for r in 0..2 {
            for c in 0..6 {
                assert!(
                    approx(j[(r, c)], a[(r, c)], 1e-9),
                    "column {c} row {r}: got {}, want {}",
                    j[(r, c)],
                    a[(r, c)]
                );
            }
        }
    }

    /// A sample failure must name its column. A Jacobian silently missing a
    /// direction is not a Jacobian, and the covariance it produces would be
    /// confidently wrong in exactly that direction.
    #[test]
    fn a_failed_column_is_reported_with_its_index() {
        let seed = StateVector::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0));
        let f = |s: StateVector| {
            if s.velocity[1] != 5.0 {
                return Err(UncertaintyError::SampleFailed {
                    column: None,
                    message: "left the scan gate".into(),
                });
            }
            Ok(Vector2::zeros())
        };
        match bplane_jacobian(seed, f) {
            Err(UncertaintyError::SampleFailed { column, .. }) => assert_eq!(column, Some(4)),
            other => panic!("expected a tagged column-4 failure, got {other:?}"),
        }
    }

    // --- the b-plane basis ----------------------------------------------------

    fn encounter_with_s(s: Vector3<f64>, b: Vector3<f64>, capture: f64) -> BPlaneEncounter {
        BPlaneEncounter {
            v_inf: 7.0e3,
            impact_parameter: b.norm(),
            perigee: 3.0e6,
            capture_radius: capture,
            eccentricity: 2.0,
            earth_radius: 6.378137e6,
            mu: 3.986004418e14,
            s_hat: s.normalize(),
            b_vector: b,
        }
    }

    #[test]
    fn basis_is_orthonormal_for_every_asymptote_direction() {
        // Includes the coordinate axes, where a naive "cross with x̂" seed is
        // degenerate.
        let dirs = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(-0.3, 0.95, 0.02),
            Vector3::new(1.0, 1e-15, 0.0),
        ];
        for d in dirs {
            let enc = encounter_with_s(d, Vector3::new(0.0, 0.0, 1.0), 1.0e7);
            let basis = BPlaneBasis::from_encounter(&enc);
            assert!(approx(basis.e1.norm(), 1.0, 1e-12), "e1 unit for {d:?}");
            assert!(approx(basis.e2.norm(), 1.0, 1e-12), "e2 unit for {d:?}");
            assert!(basis.e1.dot(&basis.e2).abs() < 1e-12, "e1 ⟂ e2 for {d:?}");
            assert!(basis.e1.dot(&basis.s_hat).abs() < 1e-12, "e1 ⟂ Ŝ for {d:?}");
            assert!(basis.e2.dot(&basis.s_hat).abs() < 1e-12, "e2 ⟂ Ŝ for {d:?}");
        }
    }

    /// The projection must preserve the b-vector's length, since `B` lies in the
    /// plane the frame spans. If it does not, the frame is not the b-plane's.
    #[test]
    fn projection_preserves_the_impact_parameter() {
        let s = Vector3::new(0.3, -0.5, 0.81).normalize();
        // A b-vector genuinely in the plane: any vector minus its Ŝ component.
        let raw = Vector3::new(1.0e7, 2.0e7, -0.5e7);
        let b = raw - s * raw.dot(&s);
        let enc = encounter_with_s(s, b, 1.1e7);
        let basis = BPlaneBasis::from_encounter(&enc);
        let p = basis.project(&enc);
        assert!(approx(p.norm(), b.norm(), 1e-12));
    }

    // --- the probability integral --------------------------------------------

    fn uncertainty(mean: Vector2<f64>, cov: Matrix2<f64>, capture: f64) -> BPlaneUncertainty {
        let enc = encounter_with_s(Vector3::new(0.0, 0.0, 1.0), Vector3::zeros(), capture);
        BPlaneUncertainty {
            mean,
            covariance: cov,
            basis: BPlaneBasis::from_encounter(&enc),
            capture_radius: capture,
        }
    }

    /// The isotropic, centred case has a closed form — the Rayleigh CDF
    /// `1 − exp(−R²/2σ²)`. A quadrature that cannot reproduce a known integral is
    /// not evidence of anything, so this is checked before any result from it is
    /// believed.
    #[test]
    fn centred_isotropic_probability_matches_the_rayleigh_closed_form() {
        for sigma in [1.0e3, 1.0e4, 5.0e4, 2.0e5] {
            for r in [1.0e3, 1.131e7, 5.0e4] {
                let cov = Matrix2::new(sigma * sigma, 0.0, 0.0, sigma * sigma);
                let u = uncertainty(Vector2::zeros(), cov, r);
                let p = u.impact_probability().expect("well-posed");
                let closed = 1.0 - (-(r * r) / (2.0 * sigma * sigma)).exp();
                assert!(
                    approx(p, closed, 1e-9),
                    "σ={sigma:e} R={r:e}: quadrature {p:.12e} vs closed form {closed:.12e}"
                );
            }
        }
    }

    /// The whole reason the ξ,ζ convention can stay deferred: the probability is
    /// invariant under any orthonormal change of b-plane basis. Rotate the mean and
    /// the covariance together — and reflect, which is the case a rotation-only
    /// test would miss — and the answer must not move.
    #[test]
    fn probability_is_invariant_under_rotation_and_reflection() {
        let mean = Vector2::new(8.0e6, -3.0e6);
        let cov = Matrix2::new(4.0e13, 1.2e13, 1.2e13, 9.0e12);
        let capture = 1.1311e7;
        let base = uncertainty(mean, cov, capture)
            .impact_probability()
            .expect("well-posed");
        assert!(base > 0.01 && base < 0.99, "pick a case that discriminates");

        for angle in [0.3_f64, 1.1, 2.7, 5.9] {
            for reflect in [false, true] {
                let (c, s) = (angle.cos(), angle.sin());
                let mut r = Matrix2::new(c, -s, s, c);
                if reflect {
                    // Flip the second axis: det = −1, still orthonormal.
                    r *= Matrix2::new(1.0, 0.0, 0.0, -1.0);
                }
                let p = uncertainty(r * mean, r * cov * r.transpose(), capture)
                    .impact_probability()
                    .expect("well-posed");
                assert!(
                    approx(p, base, 1e-9),
                    "angle {angle} reflect {reflect}: {p:.12e} vs {base:.12e}"
                );
            }
        }
    }

    /// The limits have to behave: a vanishing target catches nothing, an enormous
    /// one catches everything, and a mean far outside a tight ellipse is safe.
    #[test]
    fn probability_limits_and_monotonicity() {
        let cov = Matrix2::new(1.0e10, 0.0, 0.0, 4.0e9);
        let mean = Vector2::new(2.0e4, 0.0);

        let tiny = uncertainty(mean, cov, 1.0).impact_probability().unwrap();
        assert!(tiny < 1.0e-9, "a 1 m target catches essentially nothing");

        let huge = uncertainty(mean, cov, 1.0e9).impact_probability().unwrap();
        assert!(
            approx(huge, 1.0, 1e-9),
            "a target vastly larger than the ellipse catches all"
        );

        // Monotone in the capture radius.
        let mut prev = 0.0;
        for r in [1.0e3, 1.0e4, 3.0e4, 1.0e5, 1.0e6] {
            let p = uncertainty(mean, cov, r).impact_probability().unwrap();
            assert!(p >= prev, "probability must not decrease with target size");
            prev = p;
        }

        // A nominal miss far outside a tight ellipse is safe; the same miss with a
        // wide ellipse is not — the point `sigma_distance` exists to make.
        let tight = uncertainty(
            Vector2::new(5.0e7, 0.0),
            Matrix2::new(1.0e12, 0.0, 0.0, 1.0e12),
            1.1311e7,
        );
        assert!(tight.impact_probability().unwrap() < 1.0e-12);
        assert!(tight.sigma_distance().unwrap() > 40.0);

        let wide = uncertainty(
            Vector2::new(5.0e7, 0.0),
            Matrix2::new(4.0e15, 0.0, 0.0, 4.0e15),
            1.1311e7,
        );
        assert!(wide.impact_probability().unwrap() > 1.0e-3);
        assert!(wide.sigma_distance().unwrap() < 1.0);
    }

    #[test]
    fn probability_rejects_a_degenerate_covariance_or_target() {
        let singular = Matrix2::new(1.0e10, 0.0, 0.0, 0.0);
        assert_eq!(
            uncertainty(Vector2::zeros(), singular, 1.0e7).impact_probability(),
            Err(UncertaintyError::NotPositiveDefinite)
        );
        let cov = Matrix2::new(1.0e10, 0.0, 0.0, 1.0e10);
        assert!(matches!(
            uncertainty(Vector2::zeros(), cov, 0.0).impact_probability(),
            Err(UncertaintyError::InvalidCaptureRadius(_))
        ));
    }

    // --- the linear map itself ------------------------------------------------

    /// `Σ_b = J Σ Jᵀ` checked against a case with a hand-computable answer, and the
    /// elongation the along-track shape is supposed to produce.
    #[test]
    fn covariance_maps_through_the_jacobian() {
        // J picks out v_along-ish behaviour: row 0 responds only to v_x, row 1 only
        // to r_y, so Σ_b is diagonal with entries (J00 σ_vx)² and (J11 σ_ry)².
        let mut j = Matrix2x6::zeros();
        j[(0, 3)] = 1.6e8;
        j[(1, 1)] = -27.0;
        // Velocity σ down the sensitive direction, a small position σ across it —
        // the along-track-dominated shape, in miniature.
        let cov = StateCovariance::from_sigmas([1.0, 10.0, 1.0, 5.0e-5, 1.0e-9, 1.0e-9]).unwrap();
        let enc = encounter_with_s(
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(3.0e6, 0.0, 0.0),
            1.1311e7,
        );
        let basis = BPlaneBasis::from_encounter(&enc);
        let u = BPlaneUncertainty::from_jacobian(&j, &cov, &enc, basis);

        // σ_b0 = 1.6e8 · 5e-5 = 8 000 m, σ_b1 = 27 · 10 = 270 m, no cross term.
        assert!(approx(u.covariance[(0, 0)].sqrt(), 8.0e3, 1e-12));
        assert!(approx(u.covariance[(1, 1)].sqrt(), 270.0, 1e-12));
        assert!(u.covariance[(0, 1)].abs() < 1e-9 * u.covariance[(0, 0)]);

        // 29.6:1 — the cigar the along-track shape is supposed to produce, and the
        // reason a b-plane prediction is an ellipse rather than a disc.
        let (major, minor) = u.sigma_axes();
        assert!(approx(major, 8.0e3, 1e-9), "major {major}");
        assert!(approx(minor, 270.0, 1e-9), "minor {minor}");
        assert!(approx(major / minor, 8.0e3 / 270.0, 1e-9));
    }

    /// `sigma_axes` must return the *ellipse* semi-axes, largest first, for a
    /// covariance whose principal axes are not the coordinate axes.
    #[test]
    fn sigma_axes_are_the_rotated_ellipse_axes() {
        let (a, b) = (3.0e6_f64, 5.0e5_f64);
        let angle = 0.7_f64;
        let (c, s) = (angle.cos(), angle.sin());
        let r = Matrix2::new(c, -s, s, c);
        let d = Matrix2::new(a * a, 0.0, 0.0, b * b);
        let u = uncertainty(Vector2::zeros(), r * d * r.transpose(), 1.0e7);
        let (major, minor) = u.sigma_axes();
        assert!(approx(major, a, 1e-10));
        assert!(approx(minor, b, 1e-10));
    }

    /// A perfectly linear map must leave zero residual on the shell; a map with a
    /// known quadratic bend must show it, and show it at the *right* sample.
    /// A report that reads "0 %" for both is measuring nothing.
    #[test]
    fn linearity_report_separates_a_linear_map_from_a_bent_one() {
        let mut j = Matrix2x6::zeros();
        j[(0, 3)] = 1.0e8;
        j[(1, 1)] = -27.0;
        let cov = StateCovariance::from_sigmas([1.0, 10.0, 1.0, 5.0e-5, 1.0e-9, 1.0e-9]).unwrap();
        let offsets = cov.sigma_shell(3.0);

        // Exactly linear: flown == predicted, residual identically zero.
        let flown: Vec<Vector2<f64>> = offsets.iter().map(|o| j * o).collect();
        let report = LinearityReport::new(&j, &offsets, &flown, 3.0);
        assert_eq!(report.samples.len(), 12);
        assert!(report.max_relative_residual < 1e-15);
        assert!(report.holds_within(1e-12));

        // Bend the *dominant* axis by 10%: the report must see 10% of the shell's
        // own scale, at that sample.
        let scale_index = flown
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.norm().total_cmp(&b.1.norm()))
            .map(|(i, _)| i)
            .unwrap();
        let bent: Vec<Vector2<f64>> = flown
            .iter()
            .enumerate()
            .map(|(i, f)| if i == scale_index { f * 1.1 } else { *f })
            .collect();
        let report = LinearityReport::new(&j, &offsets, &bent, 3.0);
        assert_eq!(report.worst_index, scale_index);
        assert!(
            approx(report.max_relative_residual, 0.1 / 1.1, 1e-12),
            "residual {}",
            report.max_relative_residual
        );
        assert!(!report.holds_within(0.05));
        assert!(report.holds_within(0.10));
    }

    /// The trap the absolute-residual choice exists to avoid: a shell direction
    /// whose b-plane displacement is negligible must not dominate the verdict just
    /// because it bent by a large *fraction of itself*. A per-sample relative
    /// residual reports 100 % here; the honest answer is that a direction
    /// contributing a metre to a 100 km ellipse cannot make the ellipse wrong.
    #[test]
    fn a_negligible_direction_cannot_dominate_the_verdict() {
        let mut j = Matrix2x6::zeros();
        j[(0, 3)] = 1.0e8; // the dominant direction
        j[(1, 1)] = 1.0e-3; // a direction that barely moves the b-plane at all
        let cov = StateCovariance::from_sigmas([1.0, 1.0, 1.0, 5.0e-5, 1.0e-12, 1.0e-12]).unwrap();
        let offsets = cov.sigma_shell(3.0);

        // Flown == predicted everywhere *except* the negligible direction, which is
        // off by 100% of its own (tiny) displacement.
        let flown: Vec<Vector2<f64>> = offsets
            .iter()
            .map(|o| {
                let lin = j * o;
                if lin.norm() < 1.0 {
                    lin * 2.0
                } else {
                    lin
                }
            })
            .collect();
        let report = LinearityReport::new(&j, &offsets, &flown, 3.0);
        assert!(
            report.max_relative_residual < 1e-6,
            "a metre-scale wobble in a 15 km shell reported as {:.3}%",
            report.max_relative_residual * 100.0
        );
        assert!(report.holds_within(0.05));
        // The absolute residual is still there for anyone who wants it.
        assert!(report.max_residual > 0.0);
        assert!(report.shell_scale > 1.0e4);
    }

    #[test]
    fn outer_product_builds_the_rank_one_block() {
        let v = Vector3::new(0.6, -0.8, 0.0);
        let m: Matrix3<f64> = outer(v, 4.0);
        assert!(approx(m[(0, 0)], 4.0 * 0.36, 1e-12));
        assert!(approx(m[(0, 1)], 4.0 * -0.48, 1e-12));
        assert!(approx(m.trace(), 4.0, 1e-12));
    }
}
