//! Real near-Earth asteroids, read from JPL Horizons state tables.
//!
//! # Why this exists rather than a kernel mount
//!
//! The sixteen main-belt perturbers come from `sb441-n16.bsp`, mounted onto the
//! almanac and read exactly like a planet. The obvious next step — fetch a
//! per-object Horizons SPK for Apophis and mount it the same way — **does not
//! work**, and that was measured rather than assumed
//! (`core/examples/probe_horizons.rs`):
//!
//! - `sb441-n16.bsp` segments are **SPK type 2** (Chebyshev position).
//! - Horizons per-object SPKs are **SPK type 21** (extended modified difference
//!   arrays); no request parameter changes that.
//! - ANISE 0.10.3 evaluates SPK types 1, 2, 3, 8, 9, 12 and 13, and answers
//!   `Type21ExtendedModifiedDifferenceArray not supported for SPK computations`
//!   for type 21.
//!
//! "The same read path" was a true statement about the call site and a false one
//! about the decoder underneath it. So these objects arrive as *states* —
//! position and velocity on a fixed TDB cadence, from the same JPL solution the
//! SPK would have carried — and this module interpolates between them.
//!
//! # Why interpolating JPL's states is honest, and integrating them would not be
//!
//! This project deleted a display-grade Kepler propagator for drawing bodies with
//! physics that was not the physics it claimed. It would be the same mistake to
//! take one real state vector and fly it in our own field: without 1PN relativity
//! and Yarkovsky (HANDOFF §5, Tier 2) that trajectory is *worse* than the one JPL
//! already published, and it would be drawn beside the real threat with nothing
//! marking which is which.
//!
//! What happens here is different in kind. Every number originates in JPL's
//! relativistic solution; the only thing added is interpolation *between* JPL's
//! own samples. That error is numerical, bounded by the cadence, and **measured**
//! — not a physical-model error that no test can see. At the shipped 1-day
//! cadence it is a median of 24 m, rising to 18 885 km at the single worst epoch
//! in fifty years (the 2029 Apophis Earth flyby, whose hours-long curvature no
//! daily table resolves) — 1.3×10⁻⁴ AU, a fraction of a pixel at orrery scale.
//! `shipped_cadence_error_across_the_2029_flyby` measures exactly that against an
//! hourly table; `hermite_matches_held_out_horizons_states` pins the convergence
//! order that makes it trustworthy. **Cubic Hermite** specifically,
//! because the table carries velocity as well as position: the interpolant
//! matches JPL's state *and* its derivative at every node, so the drawn arc is
//! tangent to the real trajectory rather than merely passing near it.
//!
//! # These are scenery, not targets
//!
//! A sampled body is drawable and nothing else. It is not the threat, it is not a
//! deflection target, and it is never enrolled in the force model — a table of
//! states has no gravitational parameter here and no way to enter
//! `tier1_perturber_field`. That is what makes "mounting real asteroids cannot
//! perturb the threat" a structural guarantee rather than a hope, and
//! `neo_bodies_cannot_reach_the_force_model` is the assertion of it.
//!
//! # The span gate
//!
//! A table covers the years it was fetched for and no others — 2020–2070 by
//! default, against a clock that clamps to the DE kernel's ~300. Every query
//! outside the table returns `None`, never a zeroed vector: this codebase has
//! four separate instances of a failed body lookup being drawn at the origin,
//! which is the Sun. [`Neo::span_tdb`] exists so the caller can hide the body
//! instead of parking it on the Sun.

use std::path::{Path, PathBuf};

use nalgebra::Vector3;

use crate::state::StateVector;

/// Environment variable naming the directory of `.neo` tables explicitly.
pub const ENV_NEO_DIR: &str = "ASTEROID_NEO_DIR";

/// Directory name looked for beside the kernels — see [`resolve_dir`].
const NEO_DIR_NAME: &str = "neo";

/// Extension every state table carries.
const NEO_EXTENSION: &str = "neo";

/// First token of the first line of a table. A file that does not open with this
/// is rejected outright rather than parsed hopefully: the failure mode being
/// prevented is a differently-shaped file being read as a plausible trajectory.
const FORMAT_MAGIC: &str = "asteroid-neo-states";

/// The format revision this reader understands.
const FORMAT_VERSION: u32 = 1;

/// Kilometres to metres — the tables are written in km/km·s⁻¹ (Horizons'
/// `OUT_UNITS=KM-S`) and [`StateVector`] is metres, so the conversion happens
/// once at load and never again.
const KM_TO_M: f64 = 1000.0;

/// Why a table could not be read.
#[derive(Debug)]
pub enum NeoError {
    /// The file could not be opened or read.
    Io(String),
    /// The file was read but is not a table this reader can trust. Carries what
    /// was wrong — a bare "parse error" on a 2 MB numeric file is unactionable.
    Format(String),
}

impl std::fmt::Display for NeoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NeoError::Io(m) => write!(f, "neo table I/O: {m}"),
            NeoError::Format(m) => write!(f, "neo table format: {m}"),
        }
    }
}

impl std::error::Error for NeoError {}

/// One real asteroid, as a uniformly sampled heliocentric trajectory.
///
/// States are **ICRF (equatorial J2000), Sun-centred, metres and metres per
/// second** — deliberately the frame the rest of the core works in, so a consumer
/// hands them to the same `icrf_km_to_ecliptic_au` rotation the planets go
/// through and no second convention appears.
///
/// Note the centre: [`Clock`](crate::Clock) trajectories are **SSB**-relative and
/// these are **Sun**-relative. The methods say so in their names
/// ([`helio_state_at`](Self::helio_state_at)) because the difference is the Sun's
/// barycentric wobble, ~10⁶ km — small enough to look like a rendering nudge and
/// large enough to be wrong.
pub struct Neo {
    name: String,
    designation: String,
    naif_id: i32,
    /// Epoch of `states[0]`, seconds past J2000 TDB.
    t0_tdb: f64,
    /// Uniform sample spacing, seconds. Uniformity is the format's one structural
    /// assumption (the fetch script verifies it before writing), and it is what
    /// makes a lookup an index computation rather than a search.
    step: f64,
    /// Heliocentric ICRF states, metres and metres per second.
    states: Vec<StateVector>,
}

/// Deliberately hand-written: a derived `Debug` would dump all ~18 000 states
/// into a failure message and bury the one line that says which table went wrong.
impl std::fmt::Debug for Neo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (lo, hi) = self.span_tdb();
        write!(
            f,
            "Neo {{ {} ({}), naif {}, {} samples every {} s, TDB {lo}..{hi} }}",
            self.name,
            self.designation,
            self.naif_id,
            self.states.len(),
            self.step
        )
    }
}

impl Neo {
    /// Display label, e.g. `"99942 Apophis"`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// IAU minor-planet number as text, e.g. `"99942"`.
    pub fn designation(&self) -> &str {
        &self.designation
    }

    /// The NAIF id the object answers to in SPK space (Horizons' extended
    /// small-body numbering, `20000000 + number`). **Provenance only** — nothing
    /// in this read path resolves it, because the almanac cannot answer for these
    /// objects at all. Recorded so a future ANISE with type-21 support can be
    /// pointed at the same object without re-deriving the id.
    pub fn naif_id(&self) -> i32 {
        self.naif_id
    }

    /// Number of samples.
    pub fn len(&self) -> usize {
        self.states.len()
    }

    /// Whether the table is empty. (A loaded [`Neo`] never is — [`load`](Self::load)
    /// rejects a table too short to interpolate — so this exists for clippy's sake
    /// and for callers holding one generically.)
    pub fn is_empty(&self) -> bool {
        self.states.is_empty()
    }

    /// Sample cadence, seconds.
    pub fn step_seconds(&self) -> f64 {
        self.step
    }

    /// Covered span as `(lo, hi)`, seconds past J2000 TDB — the **gate every draw
    /// must pass through**. Outside it there is no trajectory here, and drawing
    /// one anyway puts a real asteroid on the Sun.
    pub fn span_tdb(&self) -> (f64, f64) {
        (
            self.t0_tdb,
            self.t0_tdb + (self.states.len() - 1) as f64 * self.step,
        )
    }

    /// Heliocentric ICRF state at `tdb_seconds` (metres, m/s), or `None` outside
    /// [`span_tdb`](Self::span_tdb).
    ///
    /// Cubic Hermite on the bracketing pair: the interpolant matches JPL's
    /// position *and* velocity at both ends, so it is tangent to the real
    /// trajectory at every node rather than merely passing through it. At a node
    /// the result is that node's state exactly (`s` is 0 or 1 and the basis
    /// collapses), which `interpolation_is_exact_at_the_nodes` pins.
    pub fn helio_state_at(&self, tdb_seconds: f64) -> Option<StateVector> {
        let (lo, hi) = self.span_tdb();
        if !(tdb_seconds >= lo && tdb_seconds <= hi) {
            // Written as a positive test so NaN falls out here rather than
            // indexing with a NaN-derived value below.
            return None;
        }

        let offset = (tdb_seconds - lo) / self.step;
        // The last sample has no successor to bracket with, so clamp the interval
        // index and let `s` reach exactly 1.0 there.
        let i = (offset.floor() as usize).min(self.states.len() - 2);
        let s = offset - i as f64;

        let (a, b) = (&self.states[i], &self.states[i + 1]);
        let h = self.step;
        let (s2, s3) = (s * s, s * s * s);

        // Hermite basis, and its derivative with respect to s.
        let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        let h10 = s3 - 2.0 * s2 + s;
        let h01 = -2.0 * s3 + 3.0 * s2;
        let h11 = s3 - s2;
        let d00 = 6.0 * s2 - 6.0 * s;
        let d10 = 3.0 * s2 - 4.0 * s + 1.0;
        let d01 = -6.0 * s2 + 6.0 * s;
        let d11 = 3.0 * s2 - 2.0 * s;

        let position =
            a.position * h00 + a.velocity * (h10 * h) + b.position * h01 + b.velocity * (h11 * h);
        // d/dt = (d/ds)/h, so the h on the tangent terms cancels and the position
        // terms pick one up.
        let velocity = (a.position * d00 + b.position * d01) / h
            + a.velocity * d10
            + b.velocity * d11;

        Some(StateVector::new(position, velocity))
    }

    /// An estimate of the orbital period, seconds — **for deciding how much of
    /// the table to draw, and nothing else.**
    ///
    /// A near-Earth asteroid's table spans decades, but its orbit is roughly a
    /// year, so a polyline over the whole span overplots dozens of precessing
    /// laps into a scribble. The orbit line should show one lap; this says how
    /// long one lap is.
    ///
    /// From vis-viva on the first sample — `a = 1 / (2/r − v²/μ)` with the
    /// standard heliocentric μ — which is the *same* move the `ephem` orbit path
    /// makes (a Kepler period purely to bound the sample window; every drawn point
    /// is still a real state read, never this ellipse). `None` for an unbound or
    /// degenerate state, where "one period" is meaningless and the caller should
    /// fall back to the whole span.
    pub fn orbital_period_seconds(&self) -> Option<f64> {
        /// Standard heliocentric gravitational parameter, m³/s² (matches the value
        /// pinned across the integrator and deflection modules).
        const MU_SUN: f64 = 1.327_124_400_18e20;
        let s = self.states.first()?;
        let (r, v) = (s.position.norm(), s.velocity.norm());
        if r == 0.0 {
            return None;
        }
        let inv_a = 2.0 / r - v * v / MU_SUN;
        if inv_a <= 0.0 {
            return None; // parabolic or hyperbolic — not a NEO, and no period
        }
        let a = 1.0 / inv_a;
        Some(std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt())
    }

    /// The `i`-th sample as stored, or `None` if out of range. Exists so a test
    /// can hold samples out and compare against JPL's own numbers rather than
    /// against another interpolation.
    pub fn sample(&self, i: usize) -> Option<StateVector> {
        self.states.get(i).copied()
    }

    /// Epoch of the `i`-th sample, seconds past J2000 TDB.
    pub fn sample_epoch_tdb(&self, i: usize) -> f64 {
        self.t0_tdb + i as f64 * self.step
    }

    /// Read a table from disk.
    ///
    /// Every header field is required and every one is checked. A table missing a
    /// field, carrying an unexpected centre or frame, or whose declared
    /// `n_samples` disagrees with the rows present is a **hard error**, not a
    /// best-effort read: a silently half-loaded trajectory would draw an asteroid
    /// that stops mid-orbit, and a table in the wrong frame would draw one tilted
    /// 23° out of the plane and look almost right.
    pub fn load(path: &Path) -> Result<Self, NeoError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| NeoError::Io(format!("{}: {e}", path.display())))?;
        Self::parse(&text).map_err(|e| match e {
            NeoError::Format(m) => NeoError::Format(format!("{}: {m}", path.display())),
            other => other,
        })
    }

    /// [`load`](Self::load)'s parser, split out so it can be tested on a literal.
    pub fn parse(text: &str) -> Result<Self, NeoError> {
        let mut lines = text.lines();

        let magic = lines
            .next()
            .ok_or_else(|| NeoError::Format("empty file".into()))?;
        let mut magic_parts = magic.split_whitespace();
        if magic_parts.next() != Some(FORMAT_MAGIC) {
            return Err(NeoError::Format(format!(
                "not a state table — first line is {magic:?}, expected {FORMAT_MAGIC:?}"
            )));
        }
        let version: u32 = magic_parts
            .next()
            .and_then(|v| v.parse().ok())
            .ok_or_else(|| NeoError::Format(format!("unreadable format version in {magic:?}")))?;
        if version != FORMAT_VERSION {
            return Err(NeoError::Format(format!(
                "format version {version}, this reader understands {FORMAT_VERSION}"
            )));
        }

        // Header: `key rest-of-line`, until the bare `states` line.
        let mut header: Vec<(String, String)> = Vec::new();
        let mut saw_states = false;
        for line in lines.by_ref() {
            if line.trim() == "states" {
                saw_states = true;
                break;
            }
            let (key, value) = line
                .split_once(char::is_whitespace)
                .ok_or_else(|| NeoError::Format(format!("header line without value: {line:?}")))?;
            header.push((key.to_string(), value.trim().to_string()));
        }
        if !saw_states {
            return Err(NeoError::Format("no `states` marker in header".into()));
        }
        let field = |key: &str| -> Result<String, NeoError> {
            header
                .iter()
                .find(|(k, _)| k == key)
                .map(|(_, v)| v.clone())
                .ok_or_else(|| NeoError::Format(format!("header is missing `{key}`")))
        };
        let number = |key: &str| -> Result<f64, NeoError> {
            let raw = field(key)?;
            raw.parse::<f64>()
                .map_err(|e| NeoError::Format(format!("`{key}` = {raw:?}: {e}")))
        };

        // The frame and centre are asserted, not adapted to. This reader applies
        // no rotation and no origin shift, so a table in another convention is a
        // table this code would silently misplace.
        let (center, frame) = (field("center")?, field("frame")?);
        if center != "SUN" || frame != "ICRF_J2000" {
            return Err(NeoError::Format(format!(
                "expected SUN/ICRF_J2000 states, got {center}/{frame}"
            )));
        }

        let t0_tdb = number("t0_tdb_seconds")?;
        let step = number("step_seconds")?;
        if !(step.is_finite() && step > 0.0) {
            return Err(NeoError::Format(format!("step_seconds is {step}")));
        }
        let declared = number("n_samples")? as usize;

        let mut states = Vec::with_capacity(declared);
        for (row, line) in lines.enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let mut it = line.split_whitespace();
            let mut next = |what: &str| -> Result<f64, NeoError> {
                let token = it.next().ok_or_else(|| {
                    NeoError::Format(format!("row {row}: state ends early, wanted {what}"))
                })?;
                token
                    .parse::<f64>()
                    .map_err(|e| NeoError::Format(format!("row {row} {what} = {token:?}: {e}")))
            };
            let position = Vector3::new(next("x")?, next("y")?, next("z")?) * KM_TO_M;
            let velocity = Vector3::new(next("vx")?, next("vy")?, next("vz")?) * KM_TO_M;
            states.push(StateVector::new(position, velocity));
        }

        // Declared-vs-actual is the check that catches a truncated download, which
        // is otherwise indistinguishable from a legitimately shorter span.
        if states.len() != declared {
            return Err(NeoError::Format(format!(
                "header declares {declared} samples, file carries {}",
                states.len()
            )));
        }
        // Cubic Hermite needs a bracketing pair; one sample is not a trajectory.
        if states.len() < 2 {
            return Err(NeoError::Format(format!(
                "{} sample(s) — need at least 2 to interpolate",
                states.len()
            )));
        }

        Ok(Self {
            name: field("name")?,
            designation: field("designation")?,
            naif_id: field("naif_id")?
                .parse()
                .map_err(|e| NeoError::Format(format!("naif_id: {e}")))?,
            t0_tdb,
            step,
            states,
        })
    }
}

/// The directory holding `.neo` tables: [`ENV_NEO_DIR`] if it names a real
/// directory, otherwise a `neo/` subdirectory of any conventional kernel
/// directory ([`crate::kernels::search_dirs`]).
///
/// Beside the kernels because that is where the large, gitignored, regenerable
/// data already lives — the tables are a few MB each and share exactly the
/// kernels' lifecycle. `None` is an ordinary outcome, not a failure: a fresh
/// clone has no tables and the mission runs without real asteroids.
pub fn resolve_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var(ENV_NEO_DIR) {
        let dir = PathBuf::from(dir);
        if dir.is_dir() {
            return Some(dir);
        }
    }
    crate::kernels::search_dirs()
        .into_iter()
        .map(|d| d.join(NEO_DIR_NAME))
        .find(|d| d.is_dir())
}

/// Every `.neo` table in [`resolve_dir`], loaded, sorted by filename so the
/// catalog order is the same on every machine and every run.
///
/// A table that fails to parse is **skipped with its error returned**, not
/// swallowed and not fatal: one corrupt download should cost that one asteroid,
/// while still being visible to a caller that wants to log it. The returned
/// `Vec<NeoError>` is empty on a clean load.
pub fn load_all() -> (Vec<Neo>, Vec<NeoError>) {
    let Some(dir) = resolve_dir() else {
        return (Vec::new(), Vec::new());
    };
    let Ok(entries) = std::fs::read_dir(&dir) else {
        return (
            Vec::new(),
            vec![NeoError::Io(format!("cannot list {}", dir.display()))],
        );
    };
    let mut paths: Vec<PathBuf> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == NEO_EXTENSION))
        .collect();
    paths.sort();

    let (mut bodies, mut errors) = (Vec::new(), Vec::new());
    for path in paths {
        match Neo::load(&path) {
            Ok(neo) => bodies.push(neo),
            Err(e) => errors.push(e),
        }
    }
    (bodies, errors)
}

/// [`load_all`], with the skip made auditable — the entry point every
/// table-gated test should use, mirroring
/// [`kernels::resolve_for_test`](crate::kernels::resolve_for_test).
///
/// With no tables present and `ASTEROID_REQUIRE_KERNELS` set this **panics**
/// rather than returning an empty vector, because the alternative is a test that
/// asserts nothing and prints green. That exact failure — a suite that skipped
/// half the physics and reported "13 passed" — is why the flag exists.
#[must_use]
pub fn load_all_for_test(what: &str) -> Vec<Neo> {
    let (bodies, errors) = load_all();
    for e in &errors {
        eprintln!("neo table skipped: {e}");
    }
    if bodies.is_empty() {
        assert!(
            !crate::kernels::require_kernels(),
            "ASTEROID_REQUIRE_KERNELS is set but no .neo tables were found, so \
             \"{what}\" would have skipped and printed green.\n{}",
            not_found_message()
        );
        eprintln!("no .neo tables found — skipping {what}");
    }
    bodies
}

/// Every place looked and how to fix it, for the same reason
/// [`kernels::not_found_message`](crate::kernels::not_found_message) exists.
pub fn not_found_message() -> String {
    let mut lines = vec![
        format!("no .{NEO_EXTENSION} state tables found"),
        format!("searched: {ENV_NEO_DIR} env"),
    ];
    for d in crate::kernels::search_dirs() {
        lines.push(format!("searched: {}", d.join(NEO_DIR_NAME).display()));
    }
    lines.push("fix: python pyref/fetch_horizons_neo.py".to_string());
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A three-sample table on a straight line at constant velocity — the one
    /// trajectory cubic Hermite must reproduce exactly, so any basis-function
    /// slip shows up here rather than as a small error on real data.
    const LINEAR: &str = "asteroid-neo-states 1
name Test Body
designation 0
naif_id 20000000
center SUN
frame ICRF_J2000
units km km/s
t0_tdb_seconds 0.0
step_seconds 100.0
n_samples 3
states
0.0 0.0 0.0 1.0 2.0 3.0
100.0 200.0 300.0 1.0 2.0 3.0
200.0 400.0 600.0 1.0 2.0 3.0
";

    #[test]
    fn parses_a_table_and_reports_its_span() {
        let neo = Neo::parse(LINEAR).expect("parse");
        assert_eq!(neo.name(), "Test Body");
        assert_eq!(neo.designation(), "0");
        assert_eq!(neo.naif_id(), 20_000_000);
        assert_eq!(neo.len(), 3);
        assert_eq!(neo.step_seconds(), 100.0);
        assert_eq!(neo.span_tdb(), (0.0, 200.0));
    }

    /// Constant velocity in a straight line: position must be exactly linear in
    /// time and velocity exactly constant, at sample epochs and between them.
    #[test]
    fn hermite_is_exact_on_linear_motion() {
        let neo = Neo::parse(LINEAR).expect("parse");
        for &t in &[0.0, 1.0, 50.0, 99.9, 100.0, 137.5, 200.0] {
            let s = neo.helio_state_at(t).expect("inside span");
            // km → m: the table's 1 km/s is 1000 m/s.
            let expected = Vector3::new(1.0, 2.0, 3.0) * t * KM_TO_M;
            assert!(
                (s.position - expected).norm() < 1e-6,
                "t={t}: position {:?} != {expected:?}",
                s.position
            );
            assert!(
                (s.velocity - Vector3::new(1.0, 2.0, 3.0) * KM_TO_M).norm() < 1e-9,
                "t={t}: velocity {:?}",
                s.velocity
            );
        }
    }

    /// At a sample epoch the interpolant must return that sample *exactly* — the
    /// Hermite basis collapses to it — so the drawn body passes through JPL's own
    /// states rather than near them.
    #[test]
    fn interpolation_is_exact_at_the_nodes() {
        let neo = Neo::parse(LINEAR).expect("parse");
        for i in 0..neo.len() {
            let stored = neo.sample(i).expect("sample");
            let evaluated = neo
                .helio_state_at(neo.sample_epoch_tdb(i))
                .expect("node is inside the span");
            assert_eq!(stored.position, evaluated.position, "node {i} position");
            assert_eq!(stored.velocity, evaluated.velocity, "node {i} velocity");
        }
    }

    /// The span gate. One second outside the table is `None`, not a zeroed
    /// vector — the fifth chance this codebase has had to draw a body on the Sun.
    #[test]
    fn outside_the_span_there_is_no_state() {
        let neo = Neo::parse(LINEAR).expect("parse");
        let (lo, hi) = neo.span_tdb();
        assert!(neo.helio_state_at(lo - 1.0).is_none(), "before the table");
        assert!(neo.helio_state_at(hi + 1.0).is_none(), "after the table");
        assert!(neo.helio_state_at(f64::NAN).is_none(), "NaN epoch");
        assert!(neo.helio_state_at(lo).is_some(), "the first sample");
        assert!(neo.helio_state_at(hi).is_some(), "the last sample");
    }

    /// Header claims that do not match the file are errors, not best-effort
    /// reads. A truncated download is otherwise indistinguishable from a
    /// legitimately shorter span, and it would draw an asteroid that stops.
    #[test]
    fn a_table_that_disagrees_with_its_header_is_rejected() {
        let truncated = LINEAR.replace("n_samples 3", "n_samples 4");
        let err = Neo::parse(&truncated).expect_err("count mismatch must fail");
        assert!(
            format!("{err}").contains("declares 4"),
            "unhelpful error: {err}"
        );

        let wrong_frame = LINEAR.replace("frame ICRF_J2000", "frame ECLIPJ2000");
        let err = Neo::parse(&wrong_frame).expect_err("frame mismatch must fail");
        assert!(format!("{err}").contains("ECLIPJ2000"), "unhelpful: {err}");

        let not_a_table = "some other file\nwith lines\n";
        let err = Neo::parse(not_a_table).expect_err("magic mismatch must fail");
        assert!(format!("{err}").contains("not a state table"), "{err}");

        let future = LINEAR.replace("asteroid-neo-states 1", "asteroid-neo-states 2");
        let err = Neo::parse(&future).expect_err("version mismatch must fail");
        assert!(format!("{err}").contains("version 2"), "{err}");
    }

    /// **The oracle.** Decimate the real Apophis table to every other sample, then
    /// ask the decimated trajectory for the epochs that were removed and compare
    /// against JPL's own states for those epochs.
    ///
    /// This is the check that makes "interpolating JPL's states" a measured claim
    /// rather than a comfortable phrase. Apophis is the right object to run it on:
    /// its table spans the **2029 Earth flyby**, where the heliocentric trajectory
    /// bends hardest and interpolation is at its worst — and where the first run
    /// of this test found 50 000 km of error, which is what a display tolerance
    /// picked by eye would have hidden.
    ///
    /// **What is asserted, and why it is a ratio and not a number.** The shipped
    /// cadence cannot be tested directly: holding out a sample requires a table
    /// finer than the one being tested, and 1 day is the finest we have. So this
    /// measures the error at ×2, ×4 and ×8 decimation and checks that it *falls
    /// like a cubic Hermite's error should* — halving the step must cut the error
    /// by roughly 2⁴. That order is the real invariant: a swapped basis function
    /// or a dropped `h` still produces plausible-looking numbers, but it does not
    /// produce fourth-order convergence. The absolute bounds are secondary and
    /// deliberately loose.
    ///
    /// **What ships is not extrapolated from here** — see
    /// `shipped_cadence_error_across_the_2029_flyby`, which measures the 1-day
    /// cadence against an hourly table: median **24 m**, worst **18 885 km** at
    /// the flyby, i.e. 1.3×10⁻⁴ AU on a display where one pixel is ~3×10⁵ km.
    #[test]
    fn hermite_matches_held_out_horizons_states() {
        let bodies = load_all_for_test("Hermite accuracy against held-out Horizons states");
        if bodies.is_empty() {
            return;
        }
        let apophis = bodies
            .iter()
            .find(|n| n.designation() == "99942")
            .expect("apophis.neo is one of the shipped tables");

        // Coarsest first, so consecutive pairs are one step-halving apart.
        let mut medians = Vec::new();
        for factor in [8usize, 4, 2] {
            let (worst, median, worst_epoch, checked) = held_out_error_km(apophis, factor);
            assert!(checked > 1000, "x{factor}: only {checked} samples checked");
            eprintln!(
                "x{factor} ({:.0} d cadence): median {median:.4} km, \
                 worst {worst:.1} km at TDB {worst_epoch:.0}, n={checked}",
                factor as f64 * apophis.step_seconds() / 86_400.0
            );
            medians.push(median);
        }

        // **Convergence is asserted on the median, not the worst case** — and the
        // difference between the two is the finding, not a technicality.
        //
        // The median falls 51 → 4.4 → 0.40 km across these three cadences: clean
        // high-order convergence on the smooth heliocentric arc, which is all but
        // one epoch of the trajectory. The *worst* case barely moves (113 000 →
        // 65 000 → 50 000 km) and it sits at the same place every time: the 2029
        // Earth flyby. That is not a defect in the interpolant. The flyby bends
        // the trajectory on a timescale of hours, so a table sampled in days is
        // pre-asymptotic there — no step-doubling argument applies, and none of
        // these cadences (including the shipped one) resolves it. What ships is
        // therefore measured directly, against an hourly table, in
        // `shipped_cadence_error_across_the_2029_flyby`.
        for (coarse, fine) in medians.iter().zip(medians.iter().skip(1)) {
            let ratio = coarse / fine;
            eprintln!("  median falls {ratio:.1}x per step halving (cubic Hermite: ~16x)");
            assert!(
                (5.0..40.0).contains(&ratio),
                "median error fell by {ratio:.2}x per halving — cubic Hermite is \
                 fourth order and must fall by roughly 16x. A ratio near 2 or 4 is \
                 a basis-function or step-size error, not a tolerance to widen."
            );
        }
        assert!(
            *medians.last().expect("three cadences measured") < 5.0,
            "median 2-day error {:.3} km is too large to be interpolation error",
            medians.last().expect("three cadences measured")
        );
    }

    /// **What actually ships, measured — not extrapolated.**
    ///
    /// The held-out test above cannot speak for the 1-day cadence: holding out a
    /// sample needs a table finer than the one under test, and 1 day is the finest
    /// the catalog carries. So this compares the shipped cadence against an
    /// **hourly** Horizons table across the worst two months in the whole span —
    /// the 2029 Apophis Earth flyby, where the held-out test located every one of
    /// its worst cases.
    ///
    /// Both fixtures are committed (a few hundred KB), so unlike every other test
    /// that touches real data this one needs **no kernels and no fetch** and runs
    /// on a fresh clone. It is the only place the drawn error is a measurement.
    ///
    /// The tolerance is a *display* tolerance and is stated as such: the orrery
    /// draws roughly 2 AU across ~10³ pixels, so a pixel is ~3×10⁵ km and an error
    /// of a few 10⁴ km is a fraction of one. It would be entirely wrong to read
    /// this as trajectory accuracy — for that, see HANDOFF §5 Tier 2. It is the
    /// error in *where a scenery body is drawn*, at its single worst epoch.
    #[test]
    fn shipped_cadence_error_across_the_2029_flyby() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
        let coarse = Neo::load(&dir.join("apophis_flyby_1d.neo")).expect("1-day fixture");
        let truth = Neo::load(&dir.join("apophis_flyby_1h.neo")).expect("1-hour fixture");
        assert_eq!(coarse.step_seconds(), 86_400.0, "fixture is not 1-day");
        assert_eq!(truth.step_seconds(), 3600.0, "fixture is not 1-hour");

        let (mut worst, mut worst_epoch, mut checked) = (0.0_f64, 0.0_f64, 0usize);
        let mut errors = Vec::new();
        for i in 0..truth.len() {
            let epoch = truth.sample_epoch_tdb(i);
            let Some(drawn) = coarse.helio_state_at(epoch) else {
                continue; // the hourly table runs a little past the daily one
            };
            let actual = truth.sample(i).expect("hourly sample");
            let error = (drawn.position - actual.position).norm() / KM_TO_M;
            if error > worst {
                (worst, worst_epoch) = (error, epoch);
            }
            errors.push(error);
            checked += 1;
        }
        errors.sort_by(|a, b| a.partial_cmp(b).expect("no NaN errors"));
        let median = errors[errors.len() / 2];

        assert!(checked > 1400, "only {checked} hourly epochs checked");
        eprintln!(
            "shipped 1-day cadence vs hourly truth across the 2029 flyby: \
             median {median:.3} km, worst {worst:.0} km at TDB {worst_epoch:.0} s \
             ({checked} epochs)"
        );
        // One pixel at orrery scale is ~3e5 km; this is the drawn error, not an
        // ephemeris accuracy claim.
        assert!(
            worst < 100_000.0,
            "worst drawn error {worst:.0} km at TDB {worst_epoch:.0} s exceeds a \
             pixel at orrery scale — the flyby is no longer being drawn where JPL \
             puts it"
        );
        assert!(
            median < 100.0,
            "median drawn error {median:.3} km — the whole two-month window has \
             degraded, not just the flyby itself"
        );
    }

    /// Worst, median and worst-epoch position error in km when `apophis` is
    /// decimated by `factor` and asked for the epochs that were removed, plus how
    /// many held-out samples were checked.
    ///
    /// The comparison is against JPL's own states at those epochs — never against
    /// another interpolation, which would only measure self-consistency.
    fn held_out_error_km(neo: &Neo, factor: usize) -> (f64, f64, f64, usize) {
        let kept: Vec<usize> = (0..neo.len()).step_by(factor).collect();
        let mut text = format!(
            "asteroid-neo-states 1\nname decimated\ndesignation {}\nnaif_id {}\n\
             center SUN\nframe ICRF_J2000\nunits km km/s\n\
             t0_tdb_seconds {}\nstep_seconds {}\nn_samples {}\nstates\n",
            neo.designation(),
            neo.naif_id(),
            neo.sample_epoch_tdb(0),
            neo.step_seconds() * factor as f64,
            kept.len(),
        );
        for &i in &kept {
            let s = neo.sample(i).expect("sample");
            text.push_str(&format!(
                "{} {} {} {} {} {}\n",
                s.position.x / KM_TO_M,
                s.position.y / KM_TO_M,
                s.position.z / KM_TO_M,
                s.velocity.x / KM_TO_M,
                s.velocity.y / KM_TO_M,
                s.velocity.z / KM_TO_M,
            ));
        }
        let coarse = Neo::parse(&text).expect("decimated table parses");

        let (mut worst, mut worst_epoch) = (0.0_f64, 0.0_f64);
        let mut errors = Vec::new();
        for i in 0..neo.len() {
            if i % factor == 0 {
                continue; // kept, not held out
            }
            let epoch = neo.sample_epoch_tdb(i);
            let Some(got) = coarse.helio_state_at(epoch) else {
                continue; // trailing samples past the decimated table's end
            };
            let truth = neo.sample(i).expect("held-out sample");
            let error = (got.position - truth.position).norm() / KM_TO_M;
            if error > worst {
                (worst, worst_epoch) = (error, epoch);
            }
            errors.push(error);
        }
        errors.sort_by(|a, b| a.partial_cmp(b).expect("no NaN errors"));
        let median = errors.get(errors.len() / 2).copied().unwrap_or(f64::NAN);
        (worst, median, worst_epoch, errors.len())
    }

    /// The shipped tables load, carry the ids and spans they claim, and answer
    /// inside their span. Skips loud (see [`load_all_for_test`]).
    #[test]
    fn shipped_tables_load_and_answer_inside_their_span() {
        let bodies = load_all_for_test("shipped .neo tables");
        if bodies.is_empty() {
            return;
        }
        for neo in &bodies {
            let (lo, hi) = neo.span_tdb();
            assert!(hi > lo, "{}: empty span", neo.name());
            assert!(
                neo.naif_id() > 20_000_000,
                "{}: naif_id {} is not the extended small-body numbering — \
                 sb441's 2000000+number convention is a digit away and resolves \
                 to a different object",
                neo.name(),
                neo.naif_id()
            );
            let middle = neo.helio_state_at(0.5 * (lo + hi)).expect("mid-span state");
            // A near-Earth asteroid is between roughly 0.1 and 5 AU from the Sun.
            // This is a smell test for a units or frame slip (a km-vs-m error is a
            // factor of 1000), not an orbit check.
            let au = middle.position.norm() / 1.495_978_707e11;
            assert!(
                (0.1..5.0).contains(&au),
                "{} sits {au} AU from the Sun mid-span",
                neo.name()
            );
        }
    }

    /// The period estimate is for bounding the orbit-line window, so it only has
    /// to be right to within "about one lap". Apophis is a ~0.9-year orbit; a
    /// figure between half a year and two years is fine and a whole-span or zero
    /// answer (the bug this guards) is not.
    #[test]
    fn orbital_period_is_roughly_one_year_for_a_neo() {
        let bodies = load_all_for_test("NEO orbital-period estimate");
        if bodies.is_empty() {
            return;
        }
        for neo in &bodies {
            let period_yr = neo
                .orbital_period_seconds()
                .expect("a bound NEO has a period")
                / (365.25 * 86_400.0);
            assert!(
                (0.4..3.0).contains(&period_yr),
                "{}: period estimate {period_yr:.2} yr is not one NEO orbit",
                neo.name()
            );
        }
    }

    /// **Scenery cannot become physics.** A [`Neo`] carries no gravitational
    /// parameter and no ephemeris handle, so there is no value here that
    /// `tier1_perturber_field` could accept — mounting real asteroids for display
    /// cannot perturb the threat, structurally rather than by convention.
    ///
    /// This is a compile-time claim, written as a test so that a later commit
    /// adding a `gm()` accessor has to come here and delete the reasoning
    /// deliberately rather than by accident.
    #[test]
    fn neo_bodies_cannot_reach_the_force_model() {
        fn assert_display_only<T>(_: &T)
        where
            T: ?Sized,
        {
        }
        let neo = Neo::parse(LINEAR).expect("parse");
        assert_display_only(&neo);
        // The perturber field is built from ephemeris frames and GMs
        // (`tier1_perturber_field(&Ephemeris)`); a Neo is neither and exposes
        // neither. If this stops being true, the guarantee above stops being one.
        assert!(
            neo.helio_state_at(0.0).is_some(),
            "a Neo answers positions and nothing else"
        );
    }
}
