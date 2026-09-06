//! Real orbit-determination covariances, read from the JPL Small-Body Database.
//!
//! [`crate::uncertainty`] maps a 6×6 covariance on the asteroid's Cartesian state
//! through the dynamics to a b-plane ellipse and an impact probability. Until
//! this module, the only covariance available to it was
//! [`StateCovariance::synthetic_along_track`] — *invented*, and labelled as such,
//! because the shipping threat is a designed rock with no observation arc. This
//! module is the other half: a covariance that came out of astrometry, for an
//! object that really exists, so the probability layer can be driven by a
//! measurement instead of by a plausible shape.
//!
//! `pyref/fetch_sbdb_covariance.py` fetches the record and writes the `.sbdb`
//! file this reads; `core/tests/fixtures/apophis.sbdb` is committed, so
//! everything here is testable with **no kernels and no network**.
//!
//! # What JPL publishes is not what a caller expects
//!
//! - **Cometary elements, not Keplerian.** The covariance is over
//!   `(e, q, tp, node, peri, i)` — eccentricity, perihelion *distance*, time of
//!   perihelion *passage*, and the three angles. There is no `a` and no `M`.
//! - **Mixed units**: dimensionless, au, days (`tp` is a Julian date, so its
//!   variance is in days²), and **degrees** for all three angles. A
//!   degrees-versus-radians slip is a factor of 3283 in a variance and the
//!   single most likely way to get this wrong.
//! - **The matrix is usually bigger than 6×6.** Apophis' is 8×8: the two
//!   non-gravitational acceleration parameters `A1`/`A2` are estimated
//!   alongside the orbit. (Bennu's carries `RHO`/`AMRAT` instead.) Dropping the
//!   trailing rows and columns *is* marginalisation for a Gaussian, so the
//!   leading 6×6 block is the correct covariance of the orbit alone with the
//!   non-gravitational uncertainty already folded into it. The fetch script
//!   records what it dropped; [`SbdbOrbit::marginalized`] reports it.
//! - **The covariance has its own epoch**, generally *not* the orbit's
//!   osculating-element epoch. Apophis' covariance is at JD 2459215.5
//!   (2020-12-17) while its elements are published at JD 2461200.5. Moving a
//!   covariance to another epoch needs a state-transition matrix we do not have,
//!   so this module works at the covariance's epoch and the file carries the
//!   element values *there* — never the ones at the orbit epoch.
//! - **Ecliptic, not ICRF.** Small-body elements are referred to the ecliptic;
//!   everything else in this crate is ICRF. [`crate::frames`] is the one
//!   rotation between them.
//!
//! # Why "validate by round-trip" would not have been a validation
//!
//! The obvious check on an element→state conversion is to convert back and
//! compare. It cannot fail usefully: the forward and reverse conversions share
//! the same unit convention, so a consistent degrees-for-radians error, or a
//! `tp` scaled by the wrong number of seconds, cancels *exactly* and the
//! round-trip passes. That is the same shape as the quantised-argmin trap
//! [`crate::uncertainty`] was built around — a plausible answer no structural
//! check rejects. Three independent gates are used instead, and each one can
//! fail on its own:
//!
//! 1. **JPL's published per-element σ against `sqrt(diag)`.** They agree
//!    exactly, which pins the ordering and the native units of the matrix
//!    before any physics happens. Checked at parse time
//!    ([`SbdbError::SigmaMismatch`]), not only in a test.
//! 2. **JPL's own Cartesian state at the covariance epoch**, carried in the file
//!    for *both* frames the conversion passes through. Reconstructing the state
//!    from the elements and comparing against the ecliptic truth pins the
//!    element conversion; comparing the rotated result against the ICRF truth
//!    pins the obliquity rotation separately. Neither is a round-trip: the
//!    right-hand side came from JPL, not from us.
//! 3. **Monte Carlo in element space against `J Σ Jᵀ`.** Draw from the element
//!    covariance, convert each draw, and compare the sample covariance of the
//!    states with the linear map. This is the gate on the Jacobian itself, and
//!    it is what says the linearisation is adequate at 1σ rather than assumed.
//!
//! A fourth, physical, check comes free: a real NEO covariance is an
//! along-track cigar, so the mapped position ellipsoid's long axis must lie
//! close to the velocity direction. An angle error breaks that visibly.
//!
//! # What this does *not* do
//!
//! It does not make the shipping campaign's impact probability real. That rock
//! is synthetic and always will be; [`StateCovariance::synthetic_along_track`]
//! remains its only honest covariance and is untouched. What this makes real is
//! the probability for an object that has an observation arc.

use std::path::Path;

use nalgebra::{Matrix3, Matrix6, Vector3};

use crate::elements::OrbitalElements;
use crate::epoch::Epoch;
use crate::frames::ecliptic_to_icrf;
use crate::keyhole::AU_M;
use crate::propagator::{solve_kepler, true_from_eccentric, PropagatorError};
use crate::state::StateVector;
use crate::uncertainty::{StateCovariance, UncertaintyError};

/// First token of the first line of a `.sbdb` file. Anything else is refused
/// outright — the failure being prevented is a differently-shaped file (or a
/// proxy's error page) being read as a plausible covariance.
pub const FORMAT_MAGIC: &str = "asteroid-sbdb-covariance";

/// The format revision this reader understands.
pub const FORMAT_VERSION: u32 = 1;

/// The element set this reader converts, in the order the SBDB delivers it.
pub const COM_LABELS: [&str; 6] = ["e", "q", "tp", "node", "peri", "i"];

/// Julian date of J2000.0 — the epoch the core counts TDB seconds from.
const JD_J2000: f64 = 2_451_545.0;

/// Seconds in a day, for `tp` (published as a Julian date, i.e. in days).
const SECONDS_PER_DAY: f64 = 86_400.0;

/// Kilometres to metres — the truth states are written in Horizons' `KM-S`.
const KM_TO_M: f64 = 1000.0;

/// How far `sqrt(diag)` may sit from the published σ before the file is refused.
///
/// They agree to every digit published for the six orbital elements, so this is
/// far looser than the check needs and still orders of magnitude tighter than
/// any real ordering or unit mistake, all of which move a variance by factors.
const SIGMA_RTOL: f64 = 1.0e-3;

/// Relative finite-difference step for the element→state Jacobian, as a fraction
/// of each element's own scale (see [`SbdbOrbit::fd_steps`]).
///
/// Unlike [`crate::uncertainty`]'s steps, this map is **closed form** — no
/// integrator, so no noise floor to sit above, and the only competition is
/// truncation against round-off. For a central difference on an analytic
/// function the relative error is `≈ u²/6 + ε/u` with `u = h/scale`, minimised
/// near `u = (3ε)^⅓ ≈ 9e-6`. Measured over Apophis' elements in
/// `probe_sbdb_covariance`, every column's `∂(r,v)/∂x` agrees with the value
/// here to better than **4e-8** relative across the four decades
/// `u ∈ [1e-7, 1e-4]`, and degrades on both sides — to 1e-2 at `u = 1e-2`
/// (truncation) and to 1e-5 at `u = 1e-9` (round-off). This sits in the middle
/// of that plateau rather than on an edge, which is the property that matters:
/// a step chosen at the edge is a Jacobian that changes when someone reformulates
/// the map slightly.
pub const FD_RELATIVE_STEP: f64 = 1.0e-6;

/// Why a `.sbdb` record could not be read or converted.
#[derive(Debug, Clone, PartialEq)]
pub enum SbdbError {
    /// The file could not be opened or read.
    Io(String),
    /// The file was read but is not a record this reader can trust. Carries what
    /// was wrong — a bare "parse error" on a matrix is unactionable.
    Format(String),
    /// The covariance's diagonal disagrees with JPL's own published per-element
    /// σ. The matrix is not the one the element block describes, which means the
    /// ordering or the units are not what this reader assumes.
    SigmaMismatch {
        /// Which element, by SBDB label.
        label: String,
        /// `sqrt(Σ_ii)` as read.
        from_diagonal: f64,
        /// The σ JPL publishes for that element.
        published: f64,
    },
    /// The elements do not describe a bound elliptical orbit, or Kepler's
    /// equation did not converge for them.
    Conversion(String),
    /// The mapped Cartesian covariance was rejected by [`StateCovariance`].
    Covariance(UncertaintyError),
}

impl std::fmt::Display for SbdbError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SbdbError::Io(m) => write!(f, "sbdb record I/O: {m}"),
            SbdbError::Format(m) => write!(f, "sbdb record format: {m}"),
            SbdbError::SigmaMismatch {
                label,
                from_diagonal,
                published,
            } => write!(
                f,
                "sbdb covariance diagonal for {label} is {from_diagonal:e} but JPL \
                 publishes σ = {published:e}: the matrix is not what the element \
                 block describes"
            ),
            SbdbError::Conversion(m) => write!(f, "sbdb element conversion: {m}"),
            SbdbError::Covariance(e) => write!(f, "sbdb mapped covariance: {e}"),
        }
    }
}

impl std::error::Error for SbdbError {}

impl From<PropagatorError> for SbdbError {
    fn from(e: PropagatorError) -> Self {
        SbdbError::Conversion(e.to_string())
    }
}

/// One real asteroid's published orbit solution and its covariance.
///
/// Elements and covariance are held in the SBDB's **native** units — the file's
/// own numbers, unconverted — so that what this type carries can be compared
/// against the published record digit for digit. Conversion to SI happens in
/// [`Self::elements_si`] and [`Self::covariance_si`], each of which says exactly
/// what it did.
#[derive(Debug, Clone, PartialEq)]
pub struct SbdbOrbit {
    name: String,
    designation: String,
    naif_id: i32,
    orbit_id: String,
    soln_date: String,
    obs_arc: String,
    marginalized: Vec<String>,
    cov_epoch_jd_tdb: f64,
    orbit_epoch_jd_tdb: f64,
    /// `(e, q[au], tp[JD TDB], node[deg], peri[deg], i[deg])` at the covariance
    /// epoch.
    elements_com: [f64; 6],
    /// JPL's published per-element σ, same order and units.
    sigmas_com: [f64; 6],
    /// The leading 6×6 block of the published covariance, native units.
    covariance_com: Matrix6<f64>,
    /// JPL's own heliocentric ecliptic-J2000 state at the covariance epoch,
    /// metres and m/s.
    truth_ecliptic: StateVector,
    /// The same state in heliocentric ICRF, metres and m/s.
    truth_icrf: StateVector,
}

/// The result of mapping an element covariance to a Cartesian one, with the
/// numerical damage the mapping did reported rather than hidden.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MappedCovariance {
    /// The covariance on the heliocentric ICRF state, metres and m/s.
    pub covariance: StateCovariance,
    /// Largest `|M_ij − M_ji|` in the raw `J Σ Jᵀ` product, before the
    /// symmetrising pass. Reported because a real orbit-determination covariance
    /// in mixed units is ill-conditioned enough that the product does not come
    /// back exactly symmetric, and a caller deserves to see how much was
    /// absorbed rather than to trust that it was small.
    pub absorbed_asymmetry: f64,
    /// The same, relative to the largest absolute entry of the product.
    pub absorbed_asymmetry_relative: f64,
}

impl SbdbOrbit {
    /// Read a `.sbdb` record from disk.
    pub fn read<P: AsRef<Path>>(path: P) -> Result<Self, SbdbError> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| SbdbError::Io(format!("{}: {e}", path.display())))?;
        Self::parse(&text)
    }

    /// Parse a `.sbdb` record from text.
    ///
    /// Every required key must be present; a missing one is an error rather than
    /// a default, because every field here is load-bearing and a defaulted epoch
    /// or frame would produce a wrong answer that looks right.
    pub fn parse(text: &str) -> Result<Self, SbdbError> {
        let mut lines = text.lines();
        let header = lines
            .next()
            .ok_or_else(|| SbdbError::Format("empty file".into()))?;
        let mut header_parts = header.split_whitespace();
        match (header_parts.next(), header_parts.next()) {
            (Some(FORMAT_MAGIC), Some(v)) if v.parse::<u32>() == Ok(FORMAT_VERSION) => {}
            (magic, version) => {
                return Err(SbdbError::Format(format!(
                    "expected `{FORMAT_MAGIC} {FORMAT_VERSION}` on line 1, found \
                     {magic:?} {version:?}"
                )))
            }
        }

        let mut kv: Vec<(String, String)> = Vec::new();
        let mut rows: Vec<Vec<f64>> = Vec::new();
        let mut in_matrix = false;
        for (n, line) in lines.enumerate() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if in_matrix {
                let row = line
                    .split_whitespace()
                    .map(|t| {
                        t.parse::<f64>()
                            .map_err(|_| SbdbError::Format(format!("line {}: {t:?}", n + 2)))
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                rows.push(row);
                continue;
            }
            if line == "covariance" {
                in_matrix = true;
                continue;
            }
            let (key, value) = line.split_once(' ').unwrap_or((line, ""));
            kv.push((key.to_string(), value.trim().to_string()));
        }

        let get = |key: &str| -> Result<&str, SbdbError> {
            kv.iter()
                .find(|(k, _)| k == key)
                .map(|(_, v)| v.as_str())
                .ok_or_else(|| SbdbError::Format(format!("missing key `{key}`")))
        };
        let floats = |key: &str, want: usize| -> Result<Vec<f64>, SbdbError> {
            let raw = get(key)?;
            let parsed = raw
                .split_whitespace()
                .map(|t| {
                    t.parse::<f64>()
                        .map_err(|_| SbdbError::Format(format!("`{key}`: bad number {t:?}")))
                })
                .collect::<Result<Vec<_>, _>>()?;
            if parsed.len() != want {
                return Err(SbdbError::Format(format!(
                    "`{key}`: expected {want} numbers, found {}",
                    parsed.len()
                )));
            }
            Ok(parsed)
        };

        // The frame and element set are the two assumptions the whole conversion
        // rests on, so they are checked rather than read.
        let element_set = get("element_set")?;
        if element_set != "COM" {
            return Err(SbdbError::Format(format!(
                "element set is `{element_set}`, not the cometary set `COM` this \
                 reader converts — its partial derivatives would be wrong"
            )));
        }
        let labels: Vec<&str> = get("labels")?.split_whitespace().collect();
        if labels != COM_LABELS {
            return Err(SbdbError::Format(format!(
                "labels are {labels:?}, not {COM_LABELS:?}"
            )));
        }
        let units: Vec<&str> = get("units")?.split_whitespace().collect();
        if units != ["none", "au", "d", "deg", "deg", "deg"] {
            return Err(SbdbError::Format(format!(
                "units are {units:?}, not the SBDB's `none au d deg deg deg`"
            )));
        }
        let frame = get("frame")?;
        if frame != "ECLIPJ2000" {
            return Err(SbdbError::Format(format!(
                "frame is `{frame}`, not `ECLIPJ2000`"
            )));
        }
        let center = get("center")?;
        if center != "SUN" {
            return Err(SbdbError::Format(format!(
                "center is `{center}`, not `SUN`"
            )));
        }
        let truth_units = get("truth_units")?;
        if truth_units != "km km/s" {
            return Err(SbdbError::Format(format!(
                "truth_units are `{truth_units}`, not `km km/s`"
            )));
        }

        if rows.len() != 6 || rows.iter().any(|r| r.len() != 6) {
            return Err(SbdbError::Format(format!(
                "covariance block is {} rows of {:?}, expected 6×6",
                rows.len(),
                rows.iter().map(Vec::len).collect::<Vec<_>>()
            )));
        }
        let mut covariance_com = Matrix6::zeros();
        for (i, row) in rows.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                if !v.is_finite() {
                    return Err(SbdbError::Format(format!(
                        "covariance[{i}][{j}] is not finite"
                    )));
                }
                covariance_com[(i, j)] = *v;
            }
        }

        let elements = floats("elements", 6)?;
        let sigmas = floats("sigmas", 6)?;
        let truth_ecl = floats("truth_state_ecliptic", 6)?;
        let truth_icrf = floats("truth_state_icrf", 6)?;

        let mut elements_com = [0.0; 6];
        let mut sigmas_com = [0.0; 6];
        elements_com.copy_from_slice(&elements);
        sigmas_com.copy_from_slice(&sigmas);

        // Gate 1, applied at load rather than only in a test: JPL's published
        // per-element σ must be `sqrt(diag)`. Anything else means the matrix and
        // the element block are not describing the same thing.
        for (i, label) in COM_LABELS.iter().enumerate() {
            let from_diagonal = covariance_com[(i, i)].sqrt();
            let published = sigmas_com[i];
            if !(published > 0.0)
                || (from_diagonal - published).abs() > SIGMA_RTOL * published
                || !from_diagonal.is_finite()
            {
                return Err(SbdbError::SigmaMismatch {
                    label: (*label).to_string(),
                    from_diagonal,
                    published,
                });
            }
        }

        let to_state = |v: &[f64]| {
            StateVector::from_components(
                v[0] * KM_TO_M,
                v[1] * KM_TO_M,
                v[2] * KM_TO_M,
                v[3] * KM_TO_M,
                v[4] * KM_TO_M,
                v[5] * KM_TO_M,
            )
        };

        let marginalized = match get("marginalized")? {
            "none" => Vec::new(),
            s => s.split_whitespace().map(str::to_string).collect(),
        };

        Ok(Self {
            name: get("name")?.to_string(),
            designation: get("designation")?.to_string(),
            naif_id: get("naif_id")?
                .parse()
                .map_err(|_| SbdbError::Format("bad naif_id".into()))?,
            orbit_id: get("orbit_id")?.to_string(),
            soln_date: get("soln_date")?.to_string(),
            obs_arc: get("obs_arc")?.to_string(),
            marginalized,
            cov_epoch_jd_tdb: floats("cov_epoch_jd_tdb", 1)?[0],
            orbit_epoch_jd_tdb: floats("orbit_epoch_jd_tdb", 1)?[0],
            elements_com,
            sigmas_com,
            covariance_com,
            truth_ecliptic: to_state(&truth_ecl),
            truth_icrf: to_state(&truth_icrf),
        })
    }

    /// Display label, e.g. `"99942 Apophis (2004 MN4)"`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// IAU minor-planet number as text.
    pub fn designation(&self) -> &str {
        &self.designation
    }

    /// The NAIF id the object answers to in SPK space. **Provenance only** —
    /// nothing here resolves it, exactly as in [`crate::horizons`].
    pub fn naif_id(&self) -> i32 {
        self.naif_id
    }

    /// JPL's orbit-solution number, e.g. `"220"`.
    pub fn orbit_id(&self) -> &str {
        &self.orbit_id
    }

    /// When JPL produced this solution.
    pub fn solution_date(&self) -> &str {
        &self.soln_date
    }

    /// `"<first obs> <last obs> <observations used>"` — the arc this covariance
    /// came out of, which is the physical reason it is the size it is.
    pub fn observation_arc(&self) -> &str {
        &self.obs_arc
    }

    /// Parameters that were estimated alongside the orbit and marginalised out
    /// by keeping only the leading 6×6 block — `["A1", "A2"]` for Apophis.
    ///
    /// Their uncertainty is *not* discarded by that: marginalising a Gaussian is
    /// exactly dropping the rows and columns, so what remains is the covariance
    /// of the orbit with the non-gravitational uncertainty folded in.
    pub fn marginalized(&self) -> &[String] {
        &self.marginalized
    }

    /// The epoch the covariance — and the elements this file carries — belong to.
    ///
    /// **Not** the orbit's osculating-element epoch ([`Self::orbit_epoch`]); see
    /// the module docs. Propagation should start here, because moving a
    /// covariance to another epoch needs a state-transition matrix.
    pub fn covariance_epoch(&self) -> Epoch {
        Epoch::from_tdb_seconds_past_j2000((self.cov_epoch_jd_tdb - JD_J2000) * SECONDS_PER_DAY)
    }

    /// The epoch JPL publishes the osculating elements at. Recorded for
    /// provenance and to make the gap visible; nothing here converts to it.
    pub fn orbit_epoch(&self) -> Epoch {
        Epoch::from_tdb_seconds_past_j2000((self.orbit_epoch_jd_tdb - JD_J2000) * SECONDS_PER_DAY)
    }

    /// The cometary elements in the SBDB's own units:
    /// `(e, q[au], tp[JD TDB], node[deg], peri[deg], i[deg])`.
    pub fn elements_com(&self) -> [f64; 6] {
        self.elements_com
    }

    /// JPL's published per-element 1σ, same order and units as
    /// [`Self::elements_com`].
    pub fn sigmas_com(&self) -> [f64; 6] {
        self.sigmas_com
    }

    /// The published covariance in the SBDB's own units — `au²`, `d²`, `deg²`
    /// and the cross terms between them. Use [`Self::covariance_si`] for
    /// anything numerical; this exists so the file can be checked against the
    /// published record.
    pub fn covariance_com(&self) -> &Matrix6<f64> {
        &self.covariance_com
    }

    /// The elements converted to this crate's units:
    /// `(e, q[m], tp[s relative to the covariance epoch], node[rad], peri[rad],
    /// i[rad])`.
    ///
    /// Every conversion is a pure scale except `tp`, which is affine: a Julian
    /// date, shifted so that **zero is the covariance epoch** and scaled to
    /// seconds. It is referred to the covariance epoch rather than to J2000 so
    /// that [`state_from_elements_si`] can be a pure function of the element vector
    /// evaluated at a fixed time — the same trick, and for the same reason, as
    /// [`crate::uncertainty`]'s fixed reduction epoch. Apophis' comes out at
    /// −9.89e6 s: perihelion was 114 days before the covariance epoch.
    ///
    /// Only the *scale* of an affine map reaches a covariance, which is why
    /// [`Self::covariance_si`] is a diagonal congruence and not something more.
    pub fn elements_si(&self) -> [f64; 6] {
        let e = self.elements_com;
        [
            e[0],
            e[1] * AU_M,
            (e[2] - self.cov_epoch_jd_tdb) * SECONDS_PER_DAY,
            e[3].to_radians(),
            e[4].to_radians(),
            e[5].to_radians(),
        ]
    }

    /// The unit scale factors taking a native element to an SI one, in order.
    fn si_scales() -> [f64; 6] {
        let deg = std::f64::consts::PI / 180.0;
        [1.0, AU_M, SECONDS_PER_DAY, deg, deg, deg]
    }

    /// The published covariance in the units of [`Self::elements_si`] — the
    /// diagonal congruence `D Σ D` with `D` the unit scales.
    ///
    /// Returned as a bare matrix, not a [`StateCovariance`]: that type validates
    /// positive-definiteness by Cholesky, and this matrix spans `1e-18` (an
    /// eccentricity variance) to `1e5` m² in the same 6×6, which no f64 Cholesky
    /// should be asked to certify. Its positive-definiteness is JPL's claim, not
    /// ours to re-derive; what we validate is the **Cartesian** covariance it
    /// maps to, whose blocks are within a representable range of each other.
    pub fn covariance_si(&self) -> Matrix6<f64> {
        let d = Self::si_scales();
        let mut m = self.covariance_com;
        for i in 0..6 {
            for j in 0..6 {
                m[(i, j)] *= d[i] * d[j];
            }
        }
        m
    }

    /// JPL's own heliocentric **ecliptic**-J2000 state at the covariance epoch,
    /// metres and m/s — the external truth for [`Self::state_ecliptic`].
    pub fn truth_state_ecliptic(&self) -> StateVector {
        self.truth_ecliptic
    }

    /// JPL's own heliocentric **ICRF** state at the covariance epoch, metres and
    /// m/s — the external truth for [`Self::state_icrf`].
    pub fn truth_state_icrf(&self) -> StateVector {
        self.truth_icrf
    }

    /// The heliocentric **ecliptic**-J2000 state these elements describe at the
    /// covariance epoch, about an attractor of gravitational parameter `mu`.
    ///
    /// `mu` must be the Sun's, in SI. The result depends on it only through the
    /// mean motion: at Apophis' 114-day interval from perihelion, the ~2e-10
    /// relative disagreement between DE440's `μ_sun` and DE441's moves the
    /// reconstructed position by ~30 m — negligible against a covariance whose
    /// own position σ is kilometres, but not zero, and worth knowing before
    /// reading a metre-level residual as an error.
    pub fn state_ecliptic(&self, mu: f64) -> Result<StateVector, SbdbError> {
        state_from_elements_si(self.elements_si(), mu)
    }

    /// The same state rotated into heliocentric **ICRF**, the frame the
    /// integrator runs in. Add the Sun's barycentric state to seed a propagation.
    pub fn state_icrf(&self, mu: f64) -> Result<StateVector, SbdbError> {
        let s = self.state_ecliptic(mu)?;
        Ok(StateVector::new(
            ecliptic_to_icrf(s.position),
            ecliptic_to_icrf(s.velocity),
        ))
    }

    /// The absolute central-difference step used for each SI element column, in
    /// that element's own SI unit.
    ///
    /// Each is [`FD_RELATIVE_STEP`] times the scale on which the map actually
    /// varies in that element, which is not the element's own magnitude:
    ///
    /// - `e` and the three angles vary on a scale of order 1 (radians for the
    ///   angles), so the step is the relative step itself. Using `e`'s own value
    ///   would make the step 5× smaller for a nearly circular orbit for no
    ///   reason.
    /// - `q` varies on the scale of `q`.
    /// - `tp` varies on the scale of the **orbital period** — the time it takes
    ///   the state to come back to itself. Not on the scale of `tp` itself,
    ///   which is an offset that can pass through zero.
    pub fn fd_steps(&self, mu: f64) -> Result<[f64; 6], SbdbError> {
        let x = self.elements_si();
        let (e, q) = (x[0], x[1]);
        if !(0.0..1.0).contains(&e) {
            return Err(SbdbError::Conversion(format!(
                "eccentricity {e} is not a bound elliptical orbit"
            )));
        }
        if !(mu > 0.0) || !mu.is_finite() {
            return Err(SbdbError::Conversion(format!("μ {mu} is not usable")));
        }
        let a = q / (1.0 - e);
        let period = std::f64::consts::TAU * (a * a * a / mu).sqrt();
        Ok([
            FD_RELATIVE_STEP,
            FD_RELATIVE_STEP * q,
            FD_RELATIVE_STEP * period,
            FD_RELATIVE_STEP,
            FD_RELATIVE_STEP,
            FD_RELATIVE_STEP,
        ])
    }

    /// The 6×6 Jacobian `∂(r, v)/∂(elements)` at the covariance epoch, in SI
    /// units throughout, in the **ecliptic** frame.
    ///
    /// Central differences with the per-column steps of [`Self::fd_steps`].
    /// Rows are `[rx, ry, rz, vx, vy, vz]`; columns are the SI elements of
    /// [`Self::elements_si`].
    pub fn jacobian_ecliptic(&self, mu: f64) -> Result<Matrix6<f64>, SbdbError> {
        let x = self.elements_si();
        let steps = self.fd_steps(mu)?;
        let mut j = Matrix6::zeros();
        for k in 0..6 {
            let h = steps[k];
            let mut plus = x;
            let mut minus = x;
            plus[k] += h;
            minus[k] -= h;
            let sp = state_from_elements_si(plus, mu)?;
            let sm = state_from_elements_si(minus, mu)?;
            for r in 0..3 {
                j[(r, k)] = (sp.position[r] - sm.position[r]) / (2.0 * h);
                j[(3 + r, k)] = (sp.velocity[r] - sm.velocity[r]) / (2.0 * h);
            }
        }
        Ok(j)
    }

    /// The published element covariance mapped to a covariance on the
    /// heliocentric **ICRF** Cartesian state at the covariance epoch — the thing
    /// [`crate::uncertainty`] consumes.
    ///
    /// Three steps, each of which can be checked on its own:
    ///
    /// 1. `Σ_si = D Σ_com D` — the unit congruence ([`Self::covariance_si`]).
    /// 2. `Σ_ecl = J Σ_si Jᵀ` — the linear map through the element→state
    ///    conversion, with `J` measured by central differences.
    /// 3. `Σ_icrf = R Σ_ecl Rᵀ` with `R` the obliquity rotation applied to both
    ///    the position and the velocity block.
    ///
    /// The product is symmetrised as `(M + Mᵀ)/2` before construction, and the
    /// asymmetry that absorbed is **reported** rather than assumed small: a real
    /// orbit-determination covariance in mixed units is ill-conditioned enough
    /// that the f64 product is not exactly symmetric, and
    /// [`StateCovariance::new`] would refuse it.
    pub fn state_covariance_icrf(&self, mu: f64) -> Result<MappedCovariance, SbdbError> {
        let j = self.jacobian_ecliptic(mu)?;
        let sigma_ecl = j * self.covariance_si() * j.transpose();

        let r = rotation_6(ecliptic_to_icrf_matrix());
        let raw = r * sigma_ecl * r.transpose();

        let mut worst = 0.0_f64;
        let mut scale = 0.0_f64;
        for i in 0..6 {
            for k in 0..6 {
                scale = scale.max(raw[(i, k)].abs());
                if k > i {
                    worst = worst.max((raw[(i, k)] - raw[(k, i)]).abs());
                }
            }
        }
        let symmetric = (raw + raw.transpose()) * 0.5;
        let covariance = StateCovariance::new(symmetric).map_err(SbdbError::Covariance)?;
        Ok(MappedCovariance {
            covariance,
            absorbed_asymmetry: worst,
            absorbed_asymmetry_relative: if scale > 0.0 { worst / scale } else { 0.0 },
        })
    }
}

/// Cometary elements in SI units → the Cartesian state they describe **at the
/// epoch their `tp` is referred to**, about an attractor of parameter `mu`.
///
/// `x = (e, q[m], tp[s relative to that epoch], node[rad], peri[rad], i[rad])`,
/// so the state comes out at `M = n·(0 − tp) = −n·tp`. Holding the evaluation
/// time fixed at the reference epoch and letting `tp` carry the offset is what
/// makes this a pure function of the element vector — which is what the Jacobian
/// differences, and the same trick as [`crate::uncertainty`]'s fixed reduction
/// epoch.
pub fn state_from_elements_si(x: [f64; 6], mu: f64) -> Result<StateVector, SbdbError> {
    let (e, q, tp, node, peri, inc) = (x[0], x[1], x[2], x[3], x[4], x[5]);
    if !(0.0..1.0).contains(&e) {
        return Err(SbdbError::Conversion(format!(
            "eccentricity {e} is not a bound elliptical orbit (0 ≤ e < 1)"
        )));
    }
    if !(q > 0.0) || !q.is_finite() || !(mu > 0.0) {
        return Err(SbdbError::Conversion(format!(
            "perihelion distance {q} m or μ {mu} is not usable"
        )));
    }
    let a = q / (1.0 - e);
    let n = (mu / (a * a * a)).sqrt();
    let mean_anomaly = n * (0.0 - tp);
    let eccentric = solve_kepler(mean_anomaly, e)?;
    let true_anomaly = true_from_eccentric(eccentric, e);
    Ok(OrbitalElements::new(a, e, inc, node, peri, true_anomaly).to_state(mu))
}

/// The ecliptic→ICRF rotation as a 3×3, built by rotating the basis vectors
/// through [`crate::frames::ecliptic_to_icrf`] so there is still exactly one
/// obliquity in the project.
fn ecliptic_to_icrf_matrix() -> Matrix3<f64> {
    Matrix3::from_columns(&[
        ecliptic_to_icrf(Vector3::x()),
        ecliptic_to_icrf(Vector3::y()),
        ecliptic_to_icrf(Vector3::z()),
    ])
}

/// A 3×3 frame rotation applied to both the position and the velocity block of a
/// 6-vector.
fn rotation_6(r: Matrix3<f64>) -> Matrix6<f64> {
    let mut m = Matrix6::zeros();
    for i in 0..3 {
        for j in 0..3 {
            m[(i, j)] = r[(i, j)];
            m[(3 + i, 3 + j)] = r[(i, j)];
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DE440's `μ_sun`, SI. The fixture's truth states came from Horizons on
    /// JPL's DE441 solution; the two `μ` agree to ~2e-10 relative, which the
    /// tolerances below account for explicitly rather than absorb.
    const MU_SUN: f64 = 1.327_124_400_18e20;

    fn apophis() -> SbdbOrbit {
        SbdbOrbit::read(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/apophis.sbdb"
        ))
        .expect("fixture reads")
    }

    #[test]
    fn the_fixture_parses_and_reports_its_provenance() {
        let o = apophis();
        assert_eq!(o.designation(), "99942");
        assert_eq!(o.naif_id(), 20_099_942);
        assert_eq!(o.orbit_id(), "220");
        assert_eq!(o.marginalized(), ["A1", "A2"]);
        // The two epochs really do differ — the fact the module exists to respect.
        assert!(
            (o.covariance_epoch().tdb_seconds_past_j2000()
                - o.orbit_epoch().tdb_seconds_past_j2000())
            .abs()
                > 365.0 * 86_400.0,
            "the covariance and orbit epochs should be years apart"
        );
    }

    /// Gate 1. The reader enforces this at load, so this test is really asking
    /// whether the *committed fixture* still satisfies it — a regenerated file
    /// from a future solution must too.
    #[test]
    fn the_diagonal_reproduces_jpls_published_sigmas() {
        let o = apophis();
        for (i, label) in COM_LABELS.iter().enumerate() {
            let from_diagonal = o.covariance_com()[(i, i)].sqrt();
            let published = o.sigmas_com()[i];
            let rel = (from_diagonal - published).abs() / published;
            assert!(rel < 1e-12, "{label}: sqrt(diag) off by {rel:e}");
        }
    }

    /// Gate 2a. The elements, converted, must reproduce JPL's own Cartesian
    /// state at the covariance epoch. This is what a round-trip could not do:
    /// the right-hand side is external, so a consistent unit error cannot cancel.
    #[test]
    fn the_elements_reproduce_jpls_ecliptic_state() {
        let o = apophis();
        let ours = o.state_ecliptic(MU_SUN).expect("converts");
        let truth = o.truth_state_ecliptic();
        let dr = (ours.position - truth.position).norm();
        let dv = (ours.velocity - truth.velocity).norm();
        // 30 m of the residual is the μ disagreement described on `state_ecliptic`;
        // the rest is the published elements' own rounding. A degrees-for-radians
        // slip would be ~1e10 m, a wrong `tp` scale ~1e9 m.
        assert!(dr < 1_000.0, "position off by {dr:.3} m");
        assert!(dv < 1e-3, "velocity off by {dv:e} m/s");
    }

    /// Gate 2b. The same state rotated must reproduce JPL's ICRF state, which
    /// pins the obliquity rotation independently of the element conversion.
    #[test]
    fn the_rotation_reproduces_jpls_icrf_state() {
        let o = apophis();
        let ours = o.state_icrf(MU_SUN).expect("converts");
        let truth = o.truth_state_icrf();
        let dr = (ours.position - truth.position).norm();
        assert!(dr < 1_000.0, "position off by {dr:.3} m");
        // The wrong obliquity sign would leave `x` alone and move `y`,`z` by
        // ~2·sin(ε)·|r| ≈ 1e11 m, so this is a very loud check.
        assert!(
            (ours.position.x - truth.position.x).abs() < 1_000.0,
            "x must survive a rotation about x"
        );
    }

    /// The mapped covariance must be the shape a real NEO's is: a cigar lying
    /// near the velocity direction. This is the physics gate — it checks that the
    /// answer is the *kind* of object orbit determination produces, which a
    /// numerical comparison against our own earlier answer could not.
    ///
    /// **Near, not along, and the gap is the interesting part.** Measured on
    /// Apophis: 655 m long by 38 m short, with the long axis **9.8° off** the
    /// velocity. That is not slop in the conversion. Taken one element at a time
    /// (`probe_sbdb_covariance`, section 3b) the contributions are far *larger*
    /// than the total — `node` alone 8 379 m, `peri` alone 9 022 m, `tp` alone
    /// 1 518 m — because the estimated element errors are strongly correlated and
    /// cancel by 14× in position space. What survives that cancellation is the
    /// node/peri residual, which sits 7.85° off the velocity, and that is what
    /// tilts the cigar. Only the pure-timing term is exactly along-track, and the
    /// probe measures it at **0.00°** — which is also an independent check on the
    /// `tp` column of the Jacobian, since a timing error can only move a body
    /// along its own path.
    ///
    /// So the tolerance here is 20°, chosen to admit the measured 9.8° with room
    /// for a different object's correlations, and to reject the failures that
    /// matter: a radians/degrees slip on the angle columns, or a frame rotation
    /// applied to one block and not the other, both of which put the long axis
    /// tens of degrees away or destroy the cigar entirely.
    #[test]
    fn the_mapped_position_ellipsoid_is_an_along_track_cigar() {
        let o = apophis();
        let mapped = o.state_covariance_icrf(MU_SUN).expect("maps");
        let m = mapped.covariance.matrix();
        let pos = m.fixed_view::<3, 3>(0, 0).into_owned();
        let eig = pos.symmetric_eigen();
        let (mut imax, mut imin) = (0, 0);
        for i in 1..3 {
            if eig.eigenvalues[i] > eig.eigenvalues[imax] {
                imax = i;
            }
            if eig.eigenvalues[i] < eig.eigenvalues[imin] {
                imin = i;
            }
        }
        let long = eig.eigenvalues[imax].sqrt();
        let short = eig.eigenvalues[imin].sqrt();
        assert!(
            long / short > 5.0,
            "a real NEO covariance is a cigar, got {long:.1} m by {short:.1} m"
        );
        let axis = eig.eigenvectors.column(imax).normalize();
        let v_hat = o.truth_state_icrf().velocity.normalize();
        let angle = axis.dot(&v_hat).abs().min(1.0).acos().to_degrees();
        assert!(
            angle < 20.0,
            "long axis is {angle:.2}° off the velocity direction; a real NEO \
             covariance does not point somewhere else"
        );
    }

    /// Gate 3, the discriminating one: the linear map `J Σ Jᵀ` against a Monte
    /// Carlo in element space. Deterministic (fixed-seed xorshift + Box–Muller,
    /// no new dependency), as `uncertainty`'s needle test already does.
    ///
    /// This is the only check that can fail on a wrong Jacobian *and* on a wrong
    /// unit scaling *and* on a linearisation that does not hold — the three ways
    /// this conversion can be quietly wrong.
    #[test]
    fn the_linear_map_matches_a_monte_carlo_in_element_space() {
        let o = apophis();
        let x0 = o.elements_si();
        let sigma = o.covariance_si();
        let chol = sigma
            .cholesky()
            .expect("JPL's covariance is positive definite");
        let l = chol.l();

        let n_draws = 20_000;
        let mut rng = Xorshift::new(0x5bdb_c0fa_1a11_ce99);
        let mut mean = nalgebra::Vector6::<f64>::zeros();
        let mut states = Vec::with_capacity(n_draws);
        for _ in 0..n_draws {
            let z = nalgebra::Vector6::from_iterator((0..6).map(|_| rng.normal()));
            let dx = l * z;
            let mut x = x0;
            for k in 0..6 {
                x[k] += dx[k];
            }
            let s = state_from_elements_si(x, MU_SUN).expect("draw converts");
            let v = nalgebra::Vector6::new(
                s.position.x,
                s.position.y,
                s.position.z,
                s.velocity.x,
                s.velocity.y,
                s.velocity.z,
            );
            mean += v;
            states.push(v);
        }
        mean /= n_draws as f64;
        let mut sample = Matrix6::<f64>::zeros();
        for v in &states {
            let d = v - mean;
            sample += d * d.transpose();
        }
        sample /= (n_draws - 1) as f64;

        let j = o.jacobian_ecliptic(MU_SUN).expect("jacobian");
        let linear = j * sigma * j.transpose();

        // Compare in *correlation-like* terms: the blocks span 1e6 m² against
        // 1e-6 m²/s², so an absolute tolerance would be meaningless. Each entry
        // is normalised by the geometric mean of the two diagonals it sits
        // between — the same normalisation a correlation matrix uses.
        let mut worst = 0.0_f64;
        for i in 0..6 {
            for k in 0..6 {
                let norm = (linear[(i, i)] * linear[(k, k)]).sqrt();
                worst = worst.max((sample[(i, k)] - linear[(i, k)]).abs() / norm);
            }
        }
        // 20 000 draws give a sampling error of order 1/√N ≈ 0.7 % on a variance,
        // so this tolerance is set by the Monte Carlo and not by the Jacobian.
        assert!(
            worst < 0.05,
            "linear map and Monte Carlo disagree by {:.2} % of the ellipse",
            worst * 100.0
        );
    }

    /// The Jacobian must not depend on the step, anywhere inside the plateau
    /// `probe_sbdb_covariance` measured. A step-dependent Jacobian means the
    /// shipping constant is on the edge of the plateau rather than in it.
    #[test]
    fn the_jacobian_is_flat_across_the_measured_step_plateau() {
        let o = apophis();
        let base = o.jacobian_ecliptic(MU_SUN).expect("jacobian");
        for factor in [0.1, 10.0] {
            let scaled = jacobian_with_relative_step(&o, FD_RELATIVE_STEP * factor);
            let mut worst = 0.0_f64;
            for i in 0..6 {
                for k in 0..6 {
                    let denom = base[(i, k)].abs().max(1e-30);
                    worst = worst.max((scaled[(i, k)] - base[(i, k)]).abs() / denom);
                }
            }
            assert!(
                worst < 1e-5,
                "step ×{factor} moved the Jacobian by {worst:e} relative"
            );
        }
    }

    /// A file whose element set is not the cometary one must be refused, not
    /// converted with the wrong partial derivatives.
    #[test]
    fn a_different_element_set_is_refused() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/apophis.sbdb"
        ))
        .unwrap();
        let equinoctial = text.replace("element_set COM", "element_set EQU");
        let err = SbdbOrbit::parse(&equinoctial).expect_err("must refuse");
        assert!(format!("{err}").contains("EQU"), "unhelpful: {err}");
    }

    /// A matrix whose diagonal no longer matches the published σ must be refused
    /// at load. Built by corrupting one entry, which is exactly the shape a
    /// wrong column ordering would have.
    #[test]
    fn a_diagonal_that_contradicts_the_published_sigma_is_refused() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/apophis.sbdb"
        ))
        .unwrap();
        let broken = text.replace("2.466289995831874e-18", "2.466289995831874e-17");
        let err = SbdbOrbit::parse(&broken).expect_err("must refuse");
        match err {
            SbdbError::SigmaMismatch { label, .. } => assert_eq!(label, "e"),
            other => panic!("wrong error: {other}"),
        }
    }

    /// Anything that is not a `.sbdb` file fails on line one — including an HTML
    /// error page, which is what a proxy returns instead of the record.
    #[test]
    fn a_foreign_file_fails_at_the_first_line() {
        let err = SbdbOrbit::parse("<!DOCTYPE html>\n<html><body>403</body></html>")
            .expect_err("must refuse");
        assert!(format!("{err}").contains(FORMAT_MAGIC), "unhelpful: {err}");
    }

    fn jacobian_with_relative_step(o: &SbdbOrbit, rel: f64) -> Matrix6<f64> {
        let x = o.elements_si();
        let base = o.fd_steps(MU_SUN).unwrap();
        let mut j = Matrix6::zeros();
        for k in 0..6 {
            let h = base[k] / FD_RELATIVE_STEP * rel;
            let (mut p, mut m) = (x, x);
            p[k] += h;
            m[k] -= h;
            let sp = state_from_elements_si(p, MU_SUN).unwrap();
            let sm = state_from_elements_si(m, MU_SUN).unwrap();
            for r in 0..3 {
                j[(r, k)] = (sp.position[r] - sm.position[r]) / (2.0 * h);
                j[(3 + r, k)] = (sp.velocity[r] - sm.velocity[r]) / (2.0 * h);
            }
        }
        j
    }

    /// A deterministic normal generator: xorshift64* plus Box–Muller. The same
    /// pattern `uncertainty`'s Monte Carlo test uses, and for the same reason —
    /// a random-seeded test that fails once a month is worse than no test.
    struct Xorshift {
        state: u64,
        spare: Option<f64>,
    }

    impl Xorshift {
        fn new(seed: u64) -> Self {
            Self {
                state: seed | 1,
                spare: None,
            }
        }

        fn next_u64(&mut self) -> u64 {
            let mut x = self.state;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.state = x;
            x.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }

        fn uniform(&mut self) -> f64 {
            // 53 bits of mantissa, open at zero so the log below is finite.
            ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        }

        fn normal(&mut self) -> f64 {
            if let Some(z) = self.spare.take() {
                return z;
            }
            let (u1, u2) = (self.uniform(), self.uniform());
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = std::f64::consts::TAU * u2;
            self.spare = Some(r * theta.sin());
            r * theta.cos()
        }
    }
}
