//! `readiness` — how long after a rock is found the first rocket can fly
//! (HANDOFF §8, Phase 3's standing defence systems).
//!
//! Every campaign number before this module let the first launch go on the first
//! day of the launch map — twelve years before impact on the shipping rock — as if
//! the rock had been found that morning and an impactor stood on the pad. The
//! deflection layer's whole thesis is that an earlier push is a cheaper one
//! (`Δv ∝ 1/lead`), so the years between *found* and *first launch* are exactly the
//! years the thesis says matter most. This module supplies them.
//!
//! # Two parts: deciding, then getting a spacecraft to the pad
//! - **Deciding.** Between discovery and the go-ahead ("authority to proceed") the
//!   orbit has to be known well enough to act on. That is set by the observations,
//!   not by engineering, so it is a property of the scenario more than of a defence
//!   system. The one sourced value: NASA's 2023 Planetary Defense Conference
//!   exercise (Barbee et al., *Planetary Defense Mission Options Analysis for the
//!   2023 PDC Hypothetical Impact Exercise Scenario*, 8th IAA PDC, April 2023,
//!   slide 2) has discovery on 10 January 2023 and the go-ahead on 1 July 2023,
//!   when the impact probability "could reach ~10%" — **172 days**.
//! - **Getting a spacecraft to the pad.** This is what a standing defence buys:
//!   - **Built from scratch.** "Typical high-reliability NASA planetary science
//!     missions require on the order of at least 48 to 60 months between the
//!     budgetary 'authorization to proceed' and launch" — Nuth, Barbee & Leung,
//!     *Defending the Earth from long-period comets and sneaky asteroids: short
//!     term threat response requires long term preparation*, J. Space Safety Eng.
//!     (2018), doi:10.1016/j.jsse.2018.07.002. The 2023 exercise calls it "the
//!     traditional ~5 year development time for interplanetary missions" (slide 7).
//!     The shipping value is the top of that range, **60 months**: "at least" makes
//!     the low end the optimistic one.
//!   - **Built in advance and stored.** The same paper: "a purpose built
//!     interceptor could be removed from storage and launched within much less than
//!     a year of receiving authorization", with launch preparation "less than a
//!     year" in its Fig. 1. "Much less than" is not a number, so the shipping value
//!     is the bound it gives, **one year** — a ready system is not credited with
//!     speed nobody has measured.
//!
//! # The cross-check: the one kinetic impactor that has flown
//! DART was approved into its design phase (Phase B) on 23 June 2017 (NASA / APL
//! release 2017-06-30) and launched on 24 November 2021: **53 months**, inside the
//! 48–60 month range. Its launch date was set by Didymos' 2022 approach, not by the
//! build, so it is a check on the range and not a measurement of the minimum. The
//! test `dart_sits_inside_the_published_build_range` pins that.
//!
//! # What the stored figure assumes, and the one stored spacecraft that flew
//! "Removed from storage and launched" assumes a rocket is there to take it. The
//! paper's own precedent says that is the hard part: Triana was built in 2000,
//! stored after the Columbia accident, revived as DSCOVR with work in 2011–2012 and
//! launched on 11 February 2015 — and "the longest delay in getting the mission to
//! L1 was in the procurement of a launch vehicle." So the one-year figure is a
//! standing system with its launcher arranged in advance, not a spacecraft on a
//! shelf. That is what [`IN_STORAGE`] means here.
//!
//! # A stock is limited: the rest are built
//! A level's `preparation_s` is when the *first* launch can go. A stored stock is
//! also a *number* of interceptors: "the availability of at least two interceptors
//! and two observer spacecraft" (Nuth, Barbee & Leung 2018, Summary) - and "the
//! interceptor would be designed to carry a nuclear device", so the two are a
//! nuclear-capable stock standing in for kinetic impactors here. [`IN_STORAGE`]
//! therefore carries `rest_preparation_s`: every launch past the stock waits for the
//! build-from-scratch delay. [`SOURCED_STOCK_SIZE`] is the paper's two; the campaign
//! layer takes the size as a dial and plans with it ([`crate::campaign::Stock`]).
//!
//! # Built ones come off a production line
//! A level's delay is when the first built impactor is ready, and the 60 months is
//! the source's figure for building *one* mission. How many more a production line
//! can finish, and how fast, nothing found here publishes: Barbee et al. 2018 (the
//! HAMMER study, Acta Astronautica 143, 37-61) sizes campaigns of 7 to 53 launches
//! and says only that building several "certainly costs time", and that the
//! infrastructure to launch many in a short time "is not currently available". So
//! the rate is the campaign's dial ([`crate::campaign::Stock::built_per_period`]),
//! with no sourced default, and [`Readiness::built_after_go_ahead`] says which
//! levels it applies to - every one but [`ON_THE_PAD`].
//!
//! # A stock already in orbit
//! A stock can also wait in orbit instead of on the ground: stacks launched to a
//! parking orbit years before anyone knew the rock was there, each with its own
//! departure stage ([`crate::station_keeping::StandingStack`]). Nothing has to be
//! launched after the go-ahead, so [`IN_ORBIT`] has no preparation delay: a stack can
//! leave on any departure date after the decision. It pays instead in mass - drag over
//! all the years it waited, and swinging its plane into line, since it could not have
//! known which way it would leave ([`crate::station_keeping`]). The stacks already flew,
//! so the launch cap does not count them. How long they waited has no source: it is
//! the caller's dial, [`IN_ORBIT_YEARS_DEFAULT`] by default. Launches past the stock are
//! built from scratch, as for [`IN_STORAGE`].
//!
//! # What this leaves to the caller
//! *When* launches can go. How many a year can follow is the campaign's own cap
//! ([`crate::campaign`]); whether the stock counts against it is the caller's
//! [`crate::campaign::StockMode`].
//! And a launch can only go where the launch map has dates: on the shipping rock the
//! map starts twelve years before impact, so a warning longer than twelve years plus
//! the delay buys no earlier launch here — not because it is worthless, but because
//! the scenario does not start earlier. It does still buy built impactors: under a
//! production line, the lots finished before the map starts are waiting at its first
//! date (the campaign layer reads the build date unclamped for exactly that).

/// Seconds in a Julian year, the unit every duration here is quoted in.
pub const YEAR_S: f64 = 365.25 * 86_400.0;

/// Discovery to go-ahead in NASA's 2023 PDC exercise, s: 10 Jan 2023 to 1 Jul 2023.
pub const PDC23_DECISION_S: f64 = 172.0 * 86_400.0;

/// The published range for building a high-reliability planetary mission, go-ahead
/// to launch, s — "at least 48 to 60 months" (Nuth, Barbee & Leung 2018).
pub const BUILD_RANGE_S: (f64, f64) = (48.0 / 12.0 * YEAR_S, 60.0 / 12.0 * YEAR_S);

/// A stored interceptor, go-ahead to launch, s — "less than a year" (Nuth, Barbee &
/// Leung 2018, Fig. 1); the bound, since "much less than" carries no number.
pub const STORED_PREPARATION_S: f64 = YEAR_S;

/// DART, Phase B approval (2017-06-23) to launch (2021-11-24), s.
pub const DART_APPROVAL_TO_LAUNCH_S: f64 = 1_615.0 * 86_400.0;

/// The stored stock the source recommends: "at least two interceptors" (Nuth,
/// Barbee & Leung 2018, Summary) - nuclear-capable ones.
pub const SOURCED_STOCK_SIZE: u32 = 2;

/// How long a stock in orbit has waited when the rock is found, years - the
/// default of a dial, not a sourced figure: no standing system has existed to say.
/// At the shipping 600 km a decade costs well under one per cent of a stack
/// ([`crate::station_keeping`]); the plane, not the years, is the larger cost.
pub const IN_ORBIT_YEARS_DEFAULT: f64 = 10.0;

/// How ready a defence is when a rock is found: the time to decide, and the time
/// from the go-ahead to the first launch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Readiness {
    /// Short label for a readout.
    pub name: &'static str,
    /// Discovery to go-ahead, s.
    pub decision_s: f64,
    /// Go-ahead to the first launch, s.
    pub preparation_s: f64,
    /// Go-ahead to the first launch *past a limited stock*, s - `None` where every
    /// launch is ready at `preparation_s` (no stock to run out of).
    pub rest_preparation_s: Option<f64>,
    /// Whether the impactors past any stock are built after the go-ahead, so how
    /// fast a production line delivers them limits the campaign
    /// ([`crate::campaign::Stock::built_per_period`]). `false` only for
    /// [`ON_THE_PAD`], the reference where every launch already stands ready.
    pub built_after_go_ahead: bool,
    /// Whether the stock waits **in orbit** ([`IN_ORBIT`]): its stacks are not
    /// launched after the go-ahead, the cap does not count them, and they pay for
    /// the wait and the plane ([`crate::station_keeping::StandingStack`]). `false`
    /// for a stock on the ground.
    pub stock_in_orbit: bool,
}

impl Readiness {
    /// Discovery to the first launch, s.
    pub fn delay_s(&self) -> f64 {
        self.decision_s + self.preparation_s
    }

    /// The first date a rocket can launch, TDB s, for a rock found `warning_s`
    /// before `impact_tdb`.
    pub fn first_launch_tdb(&self, impact_tdb: f64, warning_s: f64) -> f64 {
        impact_tdb - warning_s + self.delay_s()
    }

    /// The first date a launch past a limited stock can go, TDB s - `None` where
    /// there is no stock to run out of.
    pub fn build_from_tdb(&self, impact_tdb: f64, warning_s: f64) -> Option<f64> {
        self.rest_preparation_s
            .map(|rest| impact_tdb - warning_s + self.decision_s + rest)
    }

    /// The warning, s, at which the first launch can go at `first_launch_tdb` — the
    /// inverse of [`first_launch_tdb`](Self::first_launch_tdb). One measurement at a
    /// first-launch date answers every readiness level, each at its own warning.
    pub fn warning_for_first_launch_s(&self, impact_tdb: f64, first_launch_tdb: f64) -> f64 {
        impact_tdb - first_launch_tdb + self.delay_s()
    }
}

/// No delay at all: the rock is found and a rocket flies the same day. What every
/// campaign number assumed before this module — kept as the reference, not offered
/// as a real system.
pub const ON_THE_PAD: Readiness = Readiness {
    name: "ON THE PAD",
    decision_s: 0.0,
    preparation_s: 0.0,
    rest_preparation_s: None,
    built_after_go_ahead: false,
    stock_in_orbit: false,
};

/// A standing defence in orbit: stacks launched to a parking orbit before the rock
/// was found, waiting; one can leave on any departure after the decision, outside
/// the launch cap - and the launches past the stock are built from scratch. The
/// stacks' cost is in their mass, not in a delay (module doc).
pub const IN_ORBIT: Readiness = Readiness {
    name: "IN ORBIT",
    decision_s: PDC23_DECISION_S,
    preparation_s: 0.0,
    rest_preparation_s: Some(BUILD_RANGE_S.1),
    built_after_go_ahead: true,
    stock_in_orbit: true,
};

/// A standing defence: interceptors built in advance and stored, their launchers
/// arranged, flying a year after the go-ahead - and the launches past the stock
/// built from scratch.
pub const IN_STORAGE: Readiness = Readiness {
    name: "IN STORAGE",
    decision_s: PDC23_DECISION_S,
    preparation_s: STORED_PREPARATION_S,
    rest_preparation_s: Some(BUILD_RANGE_S.1),
    built_after_go_ahead: true,
    stock_in_orbit: false,
};

/// No standing defence: the interceptor is designed and built after the go-ahead.
/// The shipping level — it is the world as it stands.
pub const FROM_SCRATCH: Readiness = Readiness {
    name: "FROM SCRATCH",
    decision_s: PDC23_DECISION_S,
    preparation_s: BUILD_RANGE_S.1,
    rest_preparation_s: None,
    built_after_go_ahead: true,
    stock_in_orbit: false,
};

/// Every level a readout offers, the shipping one last.
pub const READINESS_LEVELS: [Readiness; 4] = [ON_THE_PAD, IN_ORBIT, IN_STORAGE, FROM_SCRATCH];

#[cfg(test)]
mod tests {
    use super::*;

    const DAY: f64 = 86_400.0;

    #[test]
    fn the_exercise_decision_time_is_its_two_dates_apart() {
        // 10 Jan to 1 Jul 2023 (not a leap year): 22 days to 1 Feb, then Feb to Jun.
        assert_eq!(
            PDC23_DECISION_S,
            (22.0 + 28.0 + 31.0 + 30.0 + 31.0 + 30.0) * DAY
        );
    }

    #[test]
    fn dart_sits_inside_the_published_build_range() {
        // 2017-06-23 to 2021-11-24: 8 days to 1 Jul, Jul to Dec 2017, three whole
        // years (2020 a leap year), then 1 Jan 2021 to 24 Nov. The published range
        // is the check, not the shipping value.
        let to_2018 = 8.0 + 31.0 + 31.0 + 30.0 + 31.0 + 30.0 + 31.0;
        let whole = 365.0 + 365.0 + 366.0;
        let in_2021 = 31.0 + 28.0 + 31.0 + 30.0 + 31.0 + 30.0 + 31.0 + 31.0 + 30.0 + 31.0 + 23.0;
        assert_eq!(DART_APPROVAL_TO_LAUNCH_S, (to_2018 + whole + in_2021) * DAY);
        assert!(DART_APPROVAL_TO_LAUNCH_S > BUILD_RANGE_S.0);
        assert!(DART_APPROVAL_TO_LAUNCH_S < BUILD_RANGE_S.1);
        let months = DART_APPROVAL_TO_LAUNCH_S / (YEAR_S / 12.0);
        assert!((months - 53.0).abs() < 0.5, "DART {months:.2} months");
    }

    #[test]
    fn the_shipping_build_is_the_slow_end_of_the_range() {
        assert_eq!(FROM_SCRATCH.preparation_s, BUILD_RANGE_S.1);
        assert!(BUILD_RANGE_S.0 < BUILD_RANGE_S.1);
        assert!(IN_STORAGE.preparation_s < BUILD_RANGE_S.0);
        assert_eq!(ON_THE_PAD.delay_s(), 0.0);
    }

    #[test]
    fn the_levels_are_ordered_fastest_first_and_ship_the_slowest() {
        let d: Vec<f64> = READINESS_LEVELS.iter().map(|r| r.delay_s()).collect();
        assert!(d.windows(2).all(|w| w[0] < w[1]), "{d:?}");
        assert_eq!(*READINESS_LEVELS.last().unwrap(), FROM_SCRATCH);
        // From scratch: 172 d to decide + 5 yr to build = 5.47 yr.
        assert!((FROM_SCRATCH.delay_s() / YEAR_S - 5.471).abs() < 1e-3);
        assert!((IN_STORAGE.delay_s() / YEAR_S - 1.471).abs() < 1e-3);
    }

    #[test]
    fn the_stock_runs_out_into_the_build_from_scratch_date() {
        let impact = 1.25e9;
        let w = 12.0 * YEAR_S;
        assert_eq!(
            IN_STORAGE.build_from_tdb(impact, w),
            Some(FROM_SCRATCH.first_launch_tdb(impact, w))
        );
        assert!(IN_STORAGE.first_launch_tdb(impact, w) < FROM_SCRATCH.first_launch_tdb(impact, w));
        assert_eq!(ON_THE_PAD.build_from_tdb(impact, w), None);
        assert_eq!(FROM_SCRATCH.build_from_tdb(impact, w), None);
        assert_eq!(SOURCED_STOCK_SIZE, 2);
        // Only the reference stands every launch ready; the others build them.
        let builds: Vec<bool> = READINESS_LEVELS
            .iter()
            .map(|r| r.built_after_go_ahead)
            .collect();
        assert_eq!(builds, [false, true, true, true]);
        // Only the in-orbit stock flies outside the cap, and it waits for nothing but
        // the decision.
        let in_orbit: Vec<bool> = READINESS_LEVELS.iter().map(|r| r.stock_in_orbit).collect();
        assert_eq!(in_orbit, [false, true, false, false]);
        assert_eq!(IN_ORBIT.delay_s(), PDC23_DECISION_S);
        assert_eq!(
            IN_ORBIT.build_from_tdb(impact, w),
            IN_STORAGE.build_from_tdb(impact, w)
        );
    }

    #[test]
    fn first_launch_and_warning_are_inverses() {
        let impact = 1.25e9;
        for r in READINESS_LEVELS {
            for warning_yr in [0.5, 3.0, 7.25, 12.0, 20.0] {
                let w = warning_yr * YEAR_S;
                let first = r.first_launch_tdb(impact, w);
                assert_eq!(first, impact - w + r.delay_s());
                let back = r.warning_for_first_launch_s(impact, first);
                assert!((back - w).abs() < 1e-6, "{} {warning_yr}", r.name);
            }
        }
        // One first-launch date, each level at its own warning: further back by
        // exactly the difference in delay.
        let first = impact - 6.0 * YEAR_S;
        let a = IN_STORAGE.warning_for_first_launch_s(impact, first);
        let b = FROM_SCRATCH.warning_for_first_launch_s(impact, first);
        assert!((b - a - (FROM_SCRATCH.delay_s() - IN_STORAGE.delay_s())).abs() < 1e-6);
    }
}
