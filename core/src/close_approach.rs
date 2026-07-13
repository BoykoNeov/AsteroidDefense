//! `close_approach` — find geocentric closest approach by root-finding the
//! clock's dense trajectory (HANDOFF §5, §10.8, §10.9).
//!
//! This is the missing half of the encounter pipeline that [`geometry`] deferred
//! to "the caller": [`BPlaneEncounter::from_relative_state`](crate::geometry::BPlaneEncounter::from_relative_state)
//! needs the asteroid's state **relative to Earth's geocentre, sampled near
//! closest approach**, and explicitly does *not* search for that point. This
//! module does the search — over a propagated [`Clock`] and an Earth-state source
//! — and hands the resulting relative state straight to the b-plane geometry via
//! [`CloseApproach::b_plane`].
//!
//! # The range-rate root
//! For a geocentric range `g(t) = |r_ast(t) − r_earth(t)|`, closest approach is a
//! local **minimum** of `g`, i.e. a root of
//!
//! ```text
//!   f(t) = r_rel(t) · v_rel(t) = d/dt ( ½ |r_rel|² )
//! ```
//!
//! where `r_rel = r_ast − r_earth`, `v_rel = v_ast − v_earth`. `f` is negative
//! while the range shrinks (approaching) and positive while it grows (receding),
//! so a **`− → +` crossing brackets a minimum**; a `+ → −` crossing is a local
//! *maximum* (farthest point) and is ignored. The detector scans `f` on a grid,
//! brackets every `− → +` crossing, and bisects it to the closest-approach epoch.
//!
//! The asteroid state `(r_ast, v_ast)` comes from the [`Clock`]'s dense output
//! (7th-order continuous extension, not linear interpolation — §10.9); the Earth
//! geocentre state `(r_earth, v_earth)` comes from a [`GeocentricState`] provider
//! (the ANISE-backed [`EphemerisPerturber`](crate::perturber_field::EphemerisPerturber)
//! in the shipping build, a closure in the kernel-free tests). Per DOP853's
//! construction the dense-output velocity is not *exactly* `d/dt` of the dense
//! position (they agree only to interpolation order), so `f` is the range-rate
//! only to that order — harmless here: it shifts the located root by far less than
//! the encounter matters, and the b-plane invariants are sampling-invariant anyway.
//!
//! # The one correctness-critical knob: [`ScanOptions::max_sample_dt`]
//! The DOP853 step size is set by *total* error, which is Sun-dominated on the
//! cruise, so the clock's segments do **not** reliably shrink during an Earth
//! approach until the body is deep in Earth's well. A grid built from segment
//! boundaries alone can therefore straddle a real perigee and **silently miss it**
//! (an approach-and-recede aliased away inside one interval — a missing entry, not
//! an error). `max_sample_dt` caps the grid spacing to defend against that: keep
//! `max_sample_dt · v_rel` well below the miss distance you need to resolve. The
//! 6-hour default is comfortable for typical NEO `v_inf` (a few km/s) but marginal
//! for a fast retrograde impactor (50–70 km/s) — tighten it for those.
//!
//! Because a multi-year arc contains many AU-scale synodic "closest approaches"
//! that are not encounters, [`ScanOptions::max_distance`] filters the returned
//! minima to those within a distance of interest.

use crate::clock::{Clock, ClockError};
use crate::epoch::Epoch;
use crate::forces::ForceError;
use crate::geometry::{BPlaneEncounter, GeometryError};
use crate::state::StateVector;

/// A source of a body's **SSB-relative state** (SI: metres, m/s) at an epoch —
/// for the close-approach detector, Earth's reconstructed geocentre. Object-safe.
///
/// Blanket-implemented for any `Fn(Epoch) -> Result<StateVector, ForceError>`, so
/// tests can pass a closure (a synthetic Earth) and the shipping build passes the
/// ANISE-backed [`EphemerisPerturber`](crate::perturber_field::EphemerisPerturber).
pub trait GeocentricState {
    /// The body's SSB-relative state at `epoch`, barycentric ICRF, SI.
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, ForceError>;
}

impl<F> GeocentricState for F
where
    F: Fn(Epoch) -> Result<StateVector, ForceError>,
{
    fn state_at(&self, epoch: Epoch) -> Result<StateVector, ForceError> {
        self(epoch)
    }
}

/// Why a close-approach scan could not be completed.
#[derive(Debug, Clone, PartialEq)]
pub enum CloseApproachError {
    /// A clock query failed (queried outside the propagated span). The scan only
    /// samples inside the covered span, so this surfaces a genuine clock bug.
    Clock(ClockError),
    /// The Earth-state provider failed (e.g. an ephemeris lookup error).
    Earth(ForceError),
    /// [`ScanOptions`] were invalid (non-positive cadence/tolerance, etc.).
    InvalidOptions(String),
}

impl std::fmt::Display for CloseApproachError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CloseApproachError::Clock(e) => write!(f, "close-approach clock query failed: {e}"),
            CloseApproachError::Earth(e) => write!(f, "close-approach Earth-state lookup failed: {e}"),
            CloseApproachError::InvalidOptions(m) => write!(f, "invalid scan options: {m}"),
        }
    }
}

impl std::error::Error for CloseApproachError {}

/// Tuning for a [`find_close_approaches`] scan.
///
/// [`Default`] gives a 6-hour grid cap, millisecond epoch tolerance, and no
/// distance filter — a sensible starting point for a typical NEO encounter clock.
/// Read the module note on [`max_sample_dt`](ScanOptions::max_sample_dt) before
/// trusting the output on a fast pass or a long arc.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanOptions {
    /// Maximum spacing of the scan grid, seconds (> 0). The correctness-critical
    /// knob (see the module note): too coarse and a fast approach-and-recede
    /// aliases away between samples and is silently missed. The integrator's own
    /// sub-step boundaries are used where finer than this.
    pub max_sample_dt: f64,
    /// Bracket width at which the CA-epoch bisection stops, seconds (> 0). The
    /// located epoch is good to about this; 1 ms is far below anything physical
    /// (an asteroid moves ~1 cm in a ms).
    pub time_tol_seconds: f64,
    /// Optional filter: keep only minima whose geocentric distance is `≤` this
    /// (metres). `None` returns every local minimum in the span — most of which,
    /// on a multi-year arc, are AU-scale synodic passes, not encounters.
    pub max_distance: Option<f64>,
}

impl Default for ScanOptions {
    fn default() -> Self {
        Self {
            max_sample_dt: 6.0 * 3600.0,
            time_tol_seconds: 1.0e-3,
            max_distance: None,
        }
    }
}

impl ScanOptions {
    fn validate(&self) -> Result<(), CloseApproachError> {
        if !(self.max_sample_dt.is_finite() && self.max_sample_dt > 0.0) {
            return Err(CloseApproachError::InvalidOptions(format!(
                "max_sample_dt must be finite and > 0 (got {})",
                self.max_sample_dt
            )));
        }
        if !(self.time_tol_seconds.is_finite() && self.time_tol_seconds > 0.0) {
            return Err(CloseApproachError::InvalidOptions(format!(
                "time_tol_seconds must be finite and > 0 (got {})",
                self.time_tol_seconds
            )));
        }
        if let Some(d) = self.max_distance {
            if !(d.is_finite() && d >= 0.0) {
                return Err(CloseApproachError::InvalidOptions(format!(
                    "max_distance must be finite and ≥ 0 (got {d})"
                )));
            }
        }
        Ok(())
    }
}

/// One geocentric closest approach found on a [`Clock`]'s trajectory.
///
/// All states are SI. [`b_plane`](CloseApproach::b_plane) reduces the
/// [`relative`](CloseApproach::relative) state to the b-plane hit geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CloseApproach {
    /// Epoch of closest approach (the range-rate root).
    pub epoch: Epoch,
    /// Asteroid state at [`epoch`](CloseApproach::epoch), SSB ICRF, from the
    /// clock's dense output.
    pub asteroid_ssb: StateVector,
    /// Earth geocentre state at [`epoch`](CloseApproach::epoch), SSB ICRF, from
    /// the [`GeocentricState`] provider.
    pub earth_ssb: StateVector,
    /// Earth-**relative** state (asteroid − Earth): position m, velocity m/s.
    /// This is the vector pair the b-plane geometry consumes.
    pub relative: StateVector,
    /// Geocentric distance at closest approach, metres (`|relative.position|`).
    pub distance: f64,
}

impl CloseApproach {
    /// Reduce this encounter to its b-plane geometry (`v_inf`, impact parameter,
    /// perigee, capture radius, hit verdict) via
    /// [`BPlaneEncounter::from_relative_state`]. `mu` is Earth's `μ⊕` (m³/s²),
    /// `earth_radius` the target `R⊕` (m).
    ///
    /// The relative state is sampled **at** closest approach. `geometry.rs` cautions
    /// that a caller might sample *near-but-not-at* CA to avoid cancellation in
    /// `v_inf = √(v² − 2μ/r)`; that caution does **not** bite for Earth in f64 (even
    /// a near-parabolic pass keeps ~12 significant digits, and it is *slow* passes,
    /// not fast ones, that are worst-conditioned), and CA is where Earth most
    /// dominates the dynamics, so the osculating hyperbola is cleanest there. We
    /// therefore sample at CA deliberately.
    pub fn b_plane(&self, mu: f64, earth_radius: f64) -> Result<BPlaneEncounter, GeometryError> {
        BPlaneEncounter::from_relative_state(
            self.relative.position,
            self.relative.velocity,
            mu,
            earth_radius,
        )
    }
}

/// The state of the asteroid, Earth, and their difference at absolute TDB second
/// `t` — the per-epoch primitive both the scan (via `f`) and the result-building
/// share.
fn sample_ca(
    clock: &Clock,
    earth: &dyn GeocentricState,
    t: f64,
) -> Result<CloseApproach, CloseApproachError> {
    let epoch = Epoch::from_tdb_seconds_past_j2000(t);
    let asteroid_ssb = clock.state_at(epoch).map_err(CloseApproachError::Clock)?;
    let earth_ssb = earth.state_at(epoch).map_err(CloseApproachError::Earth)?;
    let relative = StateVector::new(
        asteroid_ssb.position - earth_ssb.position,
        asteroid_ssb.velocity - earth_ssb.velocity,
    );
    let distance = relative.position.norm();
    Ok(CloseApproach {
        epoch,
        asteroid_ssb,
        earth_ssb,
        relative,
        distance,
    })
}

/// Range-rate `f(t) = r_rel · v_rel` at absolute TDB second `t`.
fn range_rate(
    clock: &Clock,
    earth: &dyn GeocentricState,
    t: f64,
) -> Result<f64, CloseApproachError> {
    let epoch = Epoch::from_tdb_seconds_past_j2000(t);
    let ast = clock.state_at(epoch).map_err(CloseApproachError::Clock)?;
    let e = earth.state_at(epoch).map_err(CloseApproachError::Earth)?;
    Ok((ast.position - e.position).dot(&(ast.velocity - e.velocity)))
}

/// Bisect `[a, b]` (given `f(a) < 0 ≤ f(b)`, so a range minimum is bracketed) to
/// the range-rate root, stopping when the bracket narrows below `time_tol`. Pure
/// bisection: the range-rate is smooth here, and the extra evaluations are cheap
/// next to guaranteed convergence.
fn bisect_min(
    mut a: f64,
    mut b: f64,
    time_tol: f64,
    mut f: impl FnMut(f64) -> Result<f64, CloseApproachError>,
) -> Result<f64, CloseApproachError> {
    // 100 halvings exhausts f64 resolution of any realistic bracket; the width
    // test normally stops it far sooner.
    for _ in 0..100 {
        if (b - a) <= time_tol {
            break;
        }
        let m = 0.5 * (a + b);
        if f(m)? < 0.0 {
            a = m;
        } else {
            b = m;
        }
    }
    Ok(0.5 * (a + b))
}

/// Find every geocentric closest approach on `clock`'s trajectory against the
/// `earth` state source, in epoch order (HANDOFF §10.9).
///
/// Scans the range-rate `f(t) = r_rel·v_rel` on a grid of the integrator's own
/// sub-step boundaries, subdivided so no gap exceeds
/// [`ScanOptions::max_sample_dt`], and bisects each `− → +` crossing to the CA
/// epoch. Returns one [`CloseApproach`] per bracketed minimum passing the optional
/// [`ScanOptions::max_distance`] filter — read the module note on aliasing before
/// trusting completeness on a fast pass.
///
/// The scan cost scales with `span / max_sample_dt` Earth-state lookups, so a long
/// arc with a tight cap is proportionally more expensive.
pub fn find_close_approaches(
    clock: &Clock,
    earth: &dyn GeocentricState,
    opts: ScanOptions,
) -> Result<Vec<CloseApproach>, CloseApproachError> {
    opts.validate()?;

    let segments = clock.segments();
    if segments.is_empty() {
        return Ok(Vec::new());
    }

    let f = |t: f64| range_rate(clock, earth, t);

    let mut prev_t = segments[0].lo();
    let mut prev_f = f(prev_t)?;
    let mut found = Vec::new();

    for seg in segments {
        let lo = seg.lo();
        let hi = seg.hi();
        let len = hi - lo;
        // Number of equal sub-intervals to keep the grid step ≤ max_sample_dt.
        let n_sub = (len / opts.max_sample_dt).ceil().max(1.0) as usize;
        for k in 1..=n_sub {
            // Land the last node exactly on `hi` (== the next segment's `lo`), so
            // the grid is contiguous and free of round-off gaps between segments.
            let t = if k == n_sub {
                hi
            } else {
                lo + len * (k as f64) / (n_sub as f64)
            };
            let fv = f(t)?;

            // `− → +`: the range stopped shrinking and began growing — a minimum.
            if prev_f < 0.0 && fv >= 0.0 {
                let t_ca = bisect_min(prev_t, t, opts.time_tol_seconds, &f)?;
                let ca = sample_ca(clock, earth, t_ca)?;
                let keep = match opts.max_distance {
                    Some(d) => ca.distance <= d,
                    None => true,
                };
                if keep {
                    found.push(ca);
                }
            }

            prev_t = t;
            prev_f = fv;
        }
    }

    Ok(found)
}

/// The single closest approach (minimum geocentric distance) on `clock`'s
/// trajectory, or `None` if the span contains no range minimum passing the
/// [`ScanOptions::max_distance`] filter. A convenience over
/// [`find_close_approaches`] for the common "just tell me the encounter" case.
pub fn closest_approach(
    clock: &Clock,
    earth: &dyn GeocentricState,
    opts: ScanOptions,
) -> Result<Option<CloseApproach>, CloseApproachError> {
    let mut best: Option<CloseApproach> = None;
    for ca in find_close_approaches(clock, earth, opts)? {
        let better = match &best {
            Some(b) => ca.distance < b.distance,
            None => true,
        };
        if better {
            best = Some(ca);
        }
    }
    Ok(best)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::forces::{ForceError, ForceModel};
    use crate::integrator::Dop853;
    use nalgebra::Vector3;

    /// Earth GM in SI (m³/s²), DE440-consistent — the same fixed literal the
    /// kernel-free `geometry.rs` tests use.
    const MU_EARTH: f64 = 3.986_004_356e14;
    /// Earth equatorial radius (m).
    const R_EARTH: f64 = crate::geometry::EARTH_EQUATORIAL_RADIUS_M;

    /// A zero-acceleration field: the asteroid drifts in a straight line, so the
    /// detector's bracketing/root-find can be checked against exact geometry with
    /// no physics in the way.
    struct ZeroForce;
    impl ForceModel for ZeroForce {
        fn acceleration(
            &self,
            _epoch: Epoch,
            _state: &StateVector,
        ) -> Result<Vector3<f64>, ForceError> {
            Ok(Vector3::zeros())
        }
    }

    /// An Earth pinned at the SSB origin, at rest — the synthetic geocentre for
    /// the kernel-free tests. `r_rel`/`v_rel` then equal the asteroid's own state.
    fn earth_at_origin() -> impl GeocentricState {
        |_epoch: Epoch| Ok(StateVector::new(Vector3::zeros(), Vector3::zeros()))
    }

    /// Straight-line pass against a fixed origin. With `r(t) = (x0 + v·dt, b, 0)`
    /// and `v = (v, 0, 0)`, the range minimum is where `f = r·v = 0`, i.e.
    /// `dt = −x0/v`, and the miss distance there is exactly `b`. The detector must
    /// recover both from the dense output alone.
    #[test]
    fn straight_line_pass_recovers_exact_epoch_and_miss() {
        let b = 5.0e8; // 500 000 km perpendicular miss
        let v = 8_000.0; // 8 km/s closing speed along +x
        let x0 = -3.0e9; // start well before CA (approaching)
        let t0 = 1_000.0_f64; // arbitrary non-zero seed epoch (s past J2000)
        let dt_ca = -x0 / v; // = 375 000 s

        let dop = Dop853::new();
        let field = ZeroForce;
        let epoch0 = Epoch::from_tdb_seconds_past_j2000(t0);
        let seed = StateVector::from_components(x0, b, 0.0, v, 0.0, 0.0);
        // Cadence/count chosen to bracket dt_ca comfortably (span ~ 750 000 s).
        let cadence = 30_000.0;
        let clock = Clock::propagate(&dop, &field, epoch0, seed, cadence, 25).unwrap();

        let earth = earth_at_origin();
        let cas = find_close_approaches(&clock, &earth, ScanOptions::default()).unwrap();
        assert_eq!(cas.len(), 1, "expected exactly one range minimum");
        let ca = cas[0];

        let epoch_err = (ca.epoch.tdb_seconds_past_j2000() - (t0 + dt_ca)).abs();
        assert!(epoch_err < 1e-2, "CA epoch off by {epoch_err:.3e} s");
        assert!(
            (ca.distance - b).abs() / b < 1e-9,
            "miss distance {} vs exact {b}",
            ca.distance
        );
        // At CA the relative position is perpendicular to the velocity (r·v ≈ 0,
        // normalized) and the closing x-component has cancelled to ~0.
        let cos = ca.relative.position.dot(&ca.relative.velocity)
            / (ca.relative.position.norm() * ca.relative.velocity.norm());
        assert!(cos.abs() < 1e-6, "r·v not perpendicular at CA (cos {cos:.3e})");
        assert!(ca.relative.position.x.abs() < 1e-2 * b);
    }

    /// A receding-then-approaching motion has a range *maximum*, not a minimum —
    /// a `+ → −` crossing the detector must ignore. Start moving away (`x0 > 0`,
    /// `v > 0`): `f = r·v > 0` throughout the forward span, so no minimum exists.
    #[test]
    fn receding_motion_yields_no_minimum() {
        let dop = Dop853::new();
        let field = ZeroForce;
        let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
        // Already past CA and receding: +x position, +x velocity.
        let seed = StateVector::from_components(1.0e9, 5.0e8, 0.0, 8_000.0, 0.0, 0.0);
        let clock = Clock::propagate(&dop, &field, epoch0, seed, 30_000.0, 20).unwrap();

        let earth = earth_at_origin();
        let cas = find_close_approaches(&clock, &earth, ScanOptions::default()).unwrap();
        assert!(cas.is_empty(), "receding motion should have no CA, got {cas:?}");
    }

    /// The `max_distance` filter drops a minimum whose miss exceeds the threshold,
    /// and `closest_approach` then returns `None`.
    #[test]
    fn max_distance_filters_distant_passes() {
        let b = 5.0e8;
        let dop = Dop853::new();
        let field = ZeroForce;
        let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
        let seed = StateVector::from_components(-3.0e9, b, 0.0, 8_000.0, 0.0, 0.0);
        let clock = Clock::propagate(&dop, &field, epoch0, seed, 30_000.0, 25).unwrap();
        let earth = earth_at_origin();

        // Threshold below the miss: the pass is filtered out.
        let opts = ScanOptions {
            max_distance: Some(0.5 * b),
            ..ScanOptions::default()
        };
        assert!(find_close_approaches(&clock, &earth, opts).unwrap().is_empty());
        assert!(closest_approach(&clock, &earth, opts).unwrap().is_none());

        // Threshold above the miss: it survives.
        let opts_wide = ScanOptions {
            max_distance: Some(2.0 * b),
            ..ScanOptions::default()
        };
        assert!(closest_approach(&clock, &earth, opts_wide).unwrap().is_some());
    }

    /// The end-to-end loop (§10.9 → §10.8): propagate a hyperbolic Earth flyby
    /// through the clock, let the detector find closest approach, and feed its
    /// relative state into `b_plane`. The recovered perigee and `v_inf` must match
    /// the analytic hyperbola the seed was built on — closing dense output →
    /// root-find → relative state → b-plane geometry into one check.
    #[test]
    fn two_body_flyby_recovers_perigee_and_b_plane() {
        // Seed a known hyperbola about an Earth at the origin: v_inf, r_p pinned;
        // sample it inbound (ν < 0) so the clock integrates through perigee.
        let v_inf = 8_000.0;
        let r_p = 3.0 * R_EARTH; // clean miss, perigee at 3 R⊕
        let e = 1.0 + r_p * v_inf * v_inf / MU_EARTH;
        let p = r_p * (1.0 + e);
        let nu = -1.1_f64; // inbound, well before perigee
        let (sin_nu, cos_nu) = nu.sin_cos();
        let r = p / (1.0 + e * cos_nu);
        let pos = Vector3::new(r * cos_nu, r * sin_nu, 0.0);
        let sqrt_mu_p = (MU_EARTH / p).sqrt();
        let vel = Vector3::new(-sin_nu, e + cos_nu, 0.0) * sqrt_mu_p;
        let seed = StateVector::new(pos, vel);

        // Point-mass Earth at the SSB origin drives the clock; the detector's Earth
        // source is that same origin at rest.
        let dop = Dop853::new();
        let field = PointMassGravity::new(vec![(MU_EARTH, FixedPerturber::at_origin()).into()]);
        let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);

        // The whole pass is a few thousand seconds; a fine cadence + tight sample
        // cap resolves the fast perigee sweep.
        let cadence = 120.0;
        let clock = Clock::propagate(&dop, &field, epoch0, seed, cadence, 120).unwrap();

        let earth = earth_at_origin();
        let opts = ScanOptions {
            max_sample_dt: 60.0,
            ..ScanOptions::default()
        };
        let ca = closest_approach(&clock, &earth, opts)
            .unwrap()
            .expect("a perigee minimum exists on this arc");

        // The geocentric closest-approach distance is the hyperbola's perigee.
        assert!(
            (ca.distance - r_p).abs() / r_p < 1e-6,
            "CA distance {} vs perigee {r_p}",
            ca.distance
        );

        // And the b-plane reduction recovers the seeded invariants.
        let enc = ca.b_plane(MU_EARTH, R_EARTH).unwrap();
        assert!(
            (enc.v_inf - v_inf).abs() / v_inf < 1e-6,
            "v_inf {} vs {v_inf}",
            enc.v_inf
        );
        assert!(
            (enc.perigee - r_p).abs() / r_p < 1e-6,
            "perigee {} vs {r_p}",
            enc.perigee
        );
        assert!(!enc.is_hit(), "3 R⊕ perigee is a clean miss");
    }

    #[test]
    fn invalid_options_are_rejected() {
        let dop = Dop853::new();
        let field = ZeroForce;
        let epoch0 = Epoch::from_tdb_seconds_past_j2000(0.0);
        let seed = StateVector::from_components(-1.0e9, 1.0e8, 0.0, 5_000.0, 0.0, 0.0);
        let clock = Clock::propagate(&dop, &field, epoch0, seed, 10_000.0, 5).unwrap();
        let earth = earth_at_origin();

        for bad in [
            ScanOptions {
                max_sample_dt: 0.0,
                ..ScanOptions::default()
            },
            ScanOptions {
                max_sample_dt: -1.0,
                ..ScanOptions::default()
            },
            ScanOptions {
                time_tol_seconds: 0.0,
                ..ScanOptions::default()
            },
            ScanOptions {
                max_distance: Some(-1.0),
                ..ScanOptions::default()
            },
        ] {
            assert!(matches!(
                find_close_approaches(&clock, &earth, bad),
                Err(CloseApproachError::InvalidOptions(_))
            ));
        }
    }
}
