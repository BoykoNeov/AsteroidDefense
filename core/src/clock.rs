//! `Clock` — the fixed-cadence snapshot clock over an integrated trajectory
//! (HANDOFF §4, §10.9).
//!
//! The resolved architecture is *fixed snapshot **cadence**, adaptive integration
//! **step** between snapshots* (HANDOFF §2). A [`Clock`] realises that: it drives
//! the [`Dop853`] encounter integrator forward one **cadence** interval at a time,
//! storing the exact integrated state at each cadence boundary as a **snapshot**,
//! *and* the [`DenseSegment`]s the integrator emits within each interval. A
//! snapshot query ([`Clock::snapshot`]) returns an exact integrated state; a
//! **sub-snapshot** query at an arbitrary epoch ([`Clock::state_at`]) is served
//! from the dense output — the 7th-order continuous extension — **not** linear
//! interpolation between snapshots.
//!
//! That distinction is the whole point of the batch: the encounter arc has high
//! curvature (Earth's gravity swinging the asteroid through the b-plane), and
//! linear interpolation between two snapshots visibly *lies* through it. Dense
//! output interpolates along the trajectory itself, so a query between snapshots
//! is as accurate as the integration (see the clock tests).
//!
//! # What the clock stores (and does not)
//! [`Clock::propagate`] consumes the integrator + force model and keeps only the
//! *results*: the snapshot states and the dense segments. The clock is then a
//! self-contained, queryable trajectory with no live physics dependency — the
//! continuous `position(t)` a future close-approach detector root-finds on
//! ([`Clock::segments`]).
//!
//! # Direction
//! The cadence may be **negative** — a backward clock, e.g. reconstructing
//! pre-encounter states for the rewind view. Snapshot `k` is always at
//! `epoch0 + k·cadence`; the dense segments tile the covered span in either
//! direction, and [`Clock::state_at`] accepts any epoch within that span.

use crate::epoch::Epoch;
use crate::forces::ForceModel;
use crate::integrator::{DenseSegment, Dop853, IntegratorError};
use crate::state::StateVector;

/// Failure modes of a clock query.
#[derive(Debug, Clone, PartialEq)]
pub enum ClockError {
    /// [`Clock::state_at`] was asked for an epoch outside the propagated span.
    /// The clock never extrapolates its dense output; it fails loud instead.
    OutOfRange {
        /// The queried epoch, seconds past J2000 (TDB).
        query_seconds: f64,
        /// Covered span lower bound, seconds past J2000 (TDB).
        span_lo: f64,
        /// Covered span upper bound, seconds past J2000 (TDB).
        span_hi: f64,
    },
}

impl std::fmt::Display for ClockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClockError::OutOfRange {
                query_seconds,
                span_lo,
                span_hi,
            } => write!(
                f,
                "clock query at t={query_seconds:.6} s (J2000 TDB) is outside the \
                 propagated span [{span_lo:.6}, {span_hi:.6}]"
            ),
        }
    }
}

impl std::error::Error for ClockError {}

/// A fixed-cadence snapshot clock over a [`Dop853`]-integrated trajectory.
///
/// Build one with [`Clock::propagate`]. Query exact snapshots by index
/// ([`Clock::snapshot`]) or the continuous trajectory at any epoch in the span
/// ([`Clock::state_at`], dense output).
#[derive(Debug, Clone)]
pub struct Clock {
    /// Epoch of snapshot 0 (the seed).
    epoch0: Epoch,
    /// Signed cadence: snapshot `k` is at `epoch0 + k·cadence_seconds`.
    cadence_seconds: f64,
    /// Exact integrated states at each cadence boundary; `snapshots[0]` is the
    /// seed and `snapshots.len() == n_snapshots + 1`.
    snapshots: Vec<StateVector>,
    /// Dense-output segments tiling the whole span, **sorted ascending by
    /// [`DenseSegment::lo`]** for binary search (independent of cadence sign).
    segments: Vec<DenseSegment>,
    /// Covered span, seconds past J2000 (TDB): `span_lo ≤ t ≤ span_hi`.
    span_lo: f64,
    span_hi: f64,
}

impl Clock {
    /// Propagate `state0` (at `epoch0`) with `integrator` under `force`, taking
    /// `n_snapshots` steps of `cadence_seconds` each and recording the dense
    /// output between snapshots.
    ///
    /// `cadence_seconds` must be finite and non-zero (its sign sets the
    /// direction); `n_snapshots` must be at least 1. Each cadence interval is a
    /// [`Dop853::step_dense`] call, so it lands exactly on the next snapshot
    /// boundary and pays the 3-extra-stage dense-output cost per accepted
    /// sub-step. Fails with the same [`IntegratorError`] the stepper would.
    pub fn propagate(
        integrator: &Dop853,
        force: &dyn ForceModel,
        epoch0: Epoch,
        state0: StateVector,
        cadence_seconds: f64,
        n_snapshots: u32,
    ) -> Result<Self, IntegratorError> {
        assert!(
            cadence_seconds.is_finite() && cadence_seconds != 0.0,
            "cadence_seconds must be finite and non-zero"
        );
        assert!(n_snapshots >= 1, "n_snapshots must be at least 1");

        let mut snapshots = Vec::with_capacity(n_snapshots as usize + 1);
        let mut segments: Vec<DenseSegment> = Vec::new();
        snapshots.push(state0);

        let mut epoch = epoch0;
        let mut state = state0;
        for _ in 0..n_snapshots {
            let (next, segs) = integrator.step_dense(force, epoch, &state, cadence_seconds)?;
            segments.extend(segs);
            state = next;
            epoch = epoch.shifted_by_seconds(cadence_seconds);
            snapshots.push(state);
        }

        // Sort segments ascending by lower bound so `state_at` can binary-search
        // regardless of integration direction. Forward runs are already sorted;
        // a backward run (cadence < 0) produces descending segments.
        segments.sort_by(|a, b| {
            a.lo()
                .partial_cmp(&b.lo())
                .expect("segment bounds are finite")
        });

        let t0 = epoch0.tdb_seconds_past_j2000();
        let t_end = epoch0
            .shifted_by_seconds(cadence_seconds * n_snapshots as f64)
            .tdb_seconds_past_j2000();
        let (span_lo, span_hi) = if t0 <= t_end {
            (t0, t_end)
        } else {
            (t_end, t0)
        };

        Ok(Self {
            epoch0,
            cadence_seconds,
            snapshots,
            segments,
            span_lo,
            span_hi,
        })
    }

    /// Epoch of snapshot 0 (the seed).
    pub fn epoch0(&self) -> Epoch {
        self.epoch0
    }

    /// Signed cadence (seconds); snapshot `k` is at `epoch0 + k·cadence`.
    pub fn cadence_seconds(&self) -> f64 {
        self.cadence_seconds
    }

    /// Number of cadence intervals integrated (one fewer than the snapshot count).
    pub fn n_intervals(&self) -> u32 {
        (self.snapshots.len() - 1) as u32
    }

    /// The epoch of snapshot `index` (`epoch0 + index·cadence`), for any index
    /// (not bounded to the propagated range).
    pub fn epoch_of(&self, index: u32) -> Epoch {
        self.epoch0
            .shifted_by_seconds(self.cadence_seconds * index as f64)
    }

    /// The exact integrated state at snapshot `index`, or `None` if `index` is
    /// past the last snapshot. Snapshot 0 is the seed.
    pub fn snapshot(&self, index: u32) -> Option<StateVector> {
        self.snapshots.get(index as usize).copied()
    }

    /// Covered span as `(span_lo, span_hi)` seconds past J2000 (TDB), lower bound
    /// first regardless of cadence sign. [`Clock::state_at`] accepts any epoch in
    /// this closed interval.
    pub fn covered_span(&self) -> (f64, f64) {
        (self.span_lo, self.span_hi)
    }

    /// The dense-output segments tiling the span (ascending by lower bound). The
    /// continuous `position(t)` a close-approach detector root-finds on.
    pub fn segments(&self) -> &[DenseSegment] {
        &self.segments
    }

    /// State at an arbitrary `epoch` within the propagated span, served from the
    /// dense output (7th-order continuous extension) — **not** linear
    /// interpolation between snapshots. At a snapshot boundary this returns the
    /// exact integrated state (dense eval is exact at segment endpoints).
    ///
    /// Fails with [`ClockError::OutOfRange`] outside the covered span; the clock
    /// never extrapolates the interpolant.
    pub fn state_at(&self, epoch: Epoch) -> Result<StateVector, ClockError> {
        let t = epoch.tdb_seconds_past_j2000();
        // Slack for the shared boundary between the span ends and the outer
        // segments, sized to the cadence so a query landing exactly on span_hi via
        // a slightly-different second count still resolves.
        let slack = 1e-6 * self.cadence_seconds.abs().max(1.0);
        if t < self.span_lo - slack || t > self.span_hi + slack {
            return Err(ClockError::OutOfRange {
                query_seconds: t,
                span_lo: self.span_lo,
                span_hi: self.span_hi,
            });
        }

        // The covering segment is the last one whose lower bound is ≤ t (segments
        // are contiguous and sorted). partition_point counts segments with lo ≤ t.
        let count = self.segments.partition_point(|s| s.lo() <= t);
        let idx = count.saturating_sub(1);
        Ok(self.segments[idx].eval(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::integrator::Integrator;
    use crate::state::StateVector;

    /// Sun gravitational parameter, SI (m³/s²).
    const MU_SUN: f64 = 1.327_124_400_18e20;
    /// 1 AU in metres.
    const AU: f64 = 1.495_978_707e11;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    fn two_body() -> PointMassGravity {
        PointMassGravity::new(vec![(MU_SUN, FixedPerturber::at_origin()).into()])
    }

    /// A representative bound heliocentric seed state (circular-ish at 1 AU).
    fn seed() -> StateVector {
        StateVector::from_components(AU, 0.0, 0.0, 0.0, (MU_SUN / AU).sqrt(), 0.0)
    }

    #[test]
    fn snapshots_match_direct_stepping() {
        // Each snapshot must equal the state a plain Dop853 step reaches from the
        // previous snapshot over one cadence — the clock's snapshots ARE those
        // exact integrated endpoints, so this pins the snapshot bookkeeping.
        let dop = Dop853::new();
        let field = two_body();
        let cadence = 10.0 * 86_400.0; // 10 days
        let n = 6;
        let clock = Clock::propagate(&dop, &field, epoch0(), seed(), cadence, n).unwrap();

        assert_eq!(clock.n_intervals(), n);
        assert_eq!(clock.snapshot(0).unwrap(), seed());
        for k in 0..n {
            let from = clock.snapshot(k).unwrap();
            let expect = dop.step(&field, clock.epoch_of(k), &from, cadence).unwrap();
            let got = clock.snapshot(k + 1).unwrap();
            assert_eq!(got, expect, "snapshot {} mismatch", k + 1);
        }
        assert!(clock.snapshot(n + 1).is_none());
    }

    #[test]
    fn state_at_snapshot_epochs_returns_exact_snapshots() {
        // A sub-snapshot query landing exactly on a cadence boundary must return
        // that snapshot's exact state (dense eval is exact at segment endpoints).
        let dop = Dop853::new();
        let field = two_body();
        let cadence = 15.0 * 86_400.0;
        let n = 4;
        let clock = Clock::propagate(&dop, &field, epoch0(), seed(), cadence, n).unwrap();

        for k in 0..=n {
            let at = clock.state_at(clock.epoch_of(k)).unwrap();
            let snap = clock.snapshot(k).unwrap();
            let rel = (at.position - snap.position).norm() / AU;
            assert!(rel < 1e-10, "snapshot {k} boundary rel err {rel:.3e}");
        }
    }

    #[test]
    fn dense_subsnapshot_beats_linear_interpolation() {
        // The pedagogical thesis (HANDOFF §10.9): between two snapshots on a curved
        // arc, dense output tracks the trajectory while linear interpolation lies.
        // Truth is a tight-tol integration to the mid-snapshot time; the dense
        // query must be orders of magnitude closer to it than the straight-line
        // interpolation between the bracketing snapshots.
        let dop = Dop853::new();
        let field = two_body();
        // A coarse cadence (~1/12 of the orbit) so the arc between snapshots is
        // visibly curved — this is where linear interp is worst.
        let period = std::f64::consts::TAU * (AU * AU * AU / MU_SUN).sqrt();
        let cadence = period / 12.0;
        let n = 12;
        let clock = Clock::propagate(&dop, &field, epoch0(), seed(), cadence, n).unwrap();

        let truth_prop = Dop853::new().with_tolerances(1e-13, 1e-6);
        let mut max_dense = 0.0_f64;
        let mut min_linear = f64::INFINITY;
        for k in 0..n {
            // Midpoint between snapshot k and k+1.
            let mid_epoch = clock.epoch_of(k).shifted_by_seconds(0.5 * cadence);
            let truth = truth_prop
                .step(&field, epoch0(), &seed(), (k as f64 + 0.5) * cadence)
                .unwrap();

            let dense = clock.state_at(mid_epoch).unwrap();
            let a = clock.snapshot(k).unwrap();
            let b = clock.snapshot(k + 1).unwrap();
            let linear = 0.5 * (a.position + b.position);

            let dense_err = (dense.position - truth.position).norm() / AU;
            let linear_err = (linear - truth.position).norm() / AU;
            max_dense = max_dense.max(dense_err);
            min_linear = min_linear.min(linear_err);
        }
        // Dense output tracks the arc to the integration tolerance (~1e-9, the
        // default rtol/atol floor); linear interp cuts the ~30° chord and is off by
        // a chord-vs-arc gap of ~3e-2 of an AU. The ratio is the real teeth: dense
        // is ~six orders tighter than the straight line it replaces.
        assert!(max_dense < 1e-8, "worst dense err {max_dense:.3e}");
        assert!(
            min_linear > 1e4 * max_dense,
            "linear interp ({min_linear:.3e}) should be far worse than dense ({max_dense:.3e})"
        );
    }

    #[test]
    fn out_of_range_query_fails_loud() {
        let dop = Dop853::new();
        let field = two_body();
        let cadence = 5.0 * 86_400.0;
        let clock = Clock::propagate(&dop, &field, epoch0(), seed(), cadence, 3).unwrap();

        // Well before the seed epoch and well after the last snapshot.
        let before = epoch0().shifted_by_seconds(-cadence);
        let after = epoch0().shifted_by_seconds(cadence * 10.0);
        assert!(matches!(
            clock.state_at(before),
            Err(ClockError::OutOfRange { .. })
        ));
        assert!(matches!(
            clock.state_at(after),
            Err(ClockError::OutOfRange { .. })
        ));
    }

    #[test]
    fn backward_clock_covers_the_past() {
        // A negative cadence reconstructs pre-seed states. Snapshots march
        // backward; the span covers [epoch0 + n·cadence, epoch0], and a
        // sub-snapshot query in that past window resolves against the dense output.
        let dop = Dop853::new();
        let field = two_body();
        let cadence = -8.0 * 86_400.0;
        let n = 5;
        let clock = Clock::propagate(&dop, &field, epoch0(), seed(), cadence, n).unwrap();

        let (lo, hi) = clock.covered_span();
        assert!((hi - 0.0).abs() < 1e-6, "span hi should be the seed epoch");
        assert!((lo - cadence * n as f64).abs() < 1e-6);

        // Snapshot k equals a direct backward step from the seed to k·cadence.
        for k in 1..=n {
            let expect = dop
                .step(&field, epoch0(), &seed(), cadence * k as f64)
                .unwrap();
            let got = clock.snapshot(k).unwrap();
            let rel = (got.position - expect.position).norm() / AU;
            assert!(rel < 1e-9, "backward snapshot {k} rel err {rel:.3e}");
        }

        // A sub-snapshot query between two past snapshots agrees with a direct
        // integration to that epoch.
        let q = clock.epoch_of(2).shifted_by_seconds(0.5 * cadence);
        let dense = clock.state_at(q).unwrap();
        let truth = Dop853::new()
            .with_tolerances(1e-13, 1e-6)
            .step(&field, epoch0(), &seed(), (2.5) * cadence)
            .unwrap();
        let rel = (dense.position - truth.position).norm() / AU;
        assert!(rel < 1e-9, "backward sub-snapshot rel err {rel:.3e}");
    }
}
