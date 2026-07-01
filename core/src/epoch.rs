//! `Epoch` — the core's canonical instant of time.
//!
//! A thin newtype over [`hifitime::Epoch`] that pins the dynamics to one time
//! scale. All of the astrodynamics in this crate runs in **TDB**
//! (Barycentric Dynamical Time) — the independent variable of the JPL DE
//! ephemerides and the barycentric ICRF integration frame (HANDOFF §5). Wrapping
//! hifitime here gives the rest of the core a single time type it cannot
//! accidentally construct in the wrong scale (UTC leap seconds, TT, etc.), and a
//! natural seconds-past-J2000 handle for the integrator to advance.
//!
//! This is deliberately minimal: it exposes the constructors and accessors the
//! propagator (§10.4) and ephemeris queries need, and defers everything else to
//! the underlying [`hifitime::Epoch`], reachable via [`Epoch::as_hifitime`].

use hifitime::{Epoch as HEpoch, TimeScale, Unit};

/// A TDB instant. The single canonical time type for the simulation core.
///
/// Construct via [`Epoch::from_tdb_gregorian`] or
/// [`Epoch::from_tdb_seconds_past_j2000`]; recover the raw hifitime value with
/// [`Epoch::as_hifitime`] when a hifitime API (formatting, other scales) is
/// needed. Equality and ordering follow the underlying instant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Epoch(HEpoch);

impl Epoch {
    /// Build a TDB epoch from a Gregorian calendar date/time (TDB scale).
    ///
    /// The sub-second field is nanoseconds. This mirrors
    /// `hifitime::Epoch::from_gregorian(.., TimeScale::TDB)` but fixes the scale
    /// so callers cannot pass UTC/TT by mistake.
    #[allow(clippy::too_many_arguments)]
    pub fn from_tdb_gregorian(
        year: i32,
        month: u8,
        day: u8,
        hour: u8,
        minute: u8,
        second: u8,
        nanos: u32,
    ) -> Self {
        Epoch(HEpoch::from_gregorian(
            year,
            month,
            day,
            hour,
            minute,
            second,
            nanos,
            TimeScale::TDB,
        ))
    }

    /// Build a TDB epoch from a count of seconds past the J2000 TDB epoch
    /// (2000-01-01 12:00:00 TDB) — the natural independent variable the
    /// integrator advances.
    pub fn from_tdb_seconds_past_j2000(seconds: f64) -> Self {
        Epoch(HEpoch::from_tdb_seconds(seconds))
    }

    /// Seconds elapsed since the J2000 TDB epoch. Inverse of
    /// [`Epoch::from_tdb_seconds_past_j2000`].
    pub fn tdb_seconds_past_j2000(&self) -> f64 {
        self.0.to_tdb_seconds()
    }

    /// This epoch advanced by `seconds` (may be negative) — the step the
    /// integrator takes between snapshots.
    pub fn shifted_by_seconds(&self, seconds: f64) -> Self {
        Epoch(self.0 + seconds * Unit::Second)
    }

    /// The underlying [`hifitime::Epoch`], for hifitime-native operations
    /// (formatting, conversion to other time scales, ephemeris queries).
    pub fn as_hifitime(&self) -> HEpoch {
        self.0
    }
}

impl From<HEpoch> for Epoch {
    /// Adopt a raw hifitime epoch. The instant is preserved exactly; only the
    /// core's *canonical* interpretation (TDB) is attached. Callers passing an
    /// epoch built in another scale are responsible for its correctness.
    fn from(e: HEpoch) -> Self {
        Epoch(e)
    }
}

impl From<Epoch> for HEpoch {
    fn from(e: Epoch) -> Self {
        e.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seconds_past_j2000_round_trips() {
        let secs = 123_456_789.25_f64;
        let e = Epoch::from_tdb_seconds_past_j2000(secs);
        assert!((e.tdb_seconds_past_j2000() - secs).abs() < 1e-6);
    }

    #[test]
    fn j2000_gregorian_is_zero_seconds() {
        // 2000-01-01 12:00:00 TDB is the J2000 TDB epoch by definition.
        let e = Epoch::from_tdb_gregorian(2000, 1, 1, 12, 0, 0, 0);
        assert!(e.tdb_seconds_past_j2000().abs() < 1e-6);
    }

    #[test]
    fn shift_is_additive() {
        let e = Epoch::from_tdb_seconds_past_j2000(1000.0);
        let e2 = e.shifted_by_seconds(500.0);
        assert!((e2.tdb_seconds_past_j2000() - 1500.0).abs() < 1e-6);
    }
}
