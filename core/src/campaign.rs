//! `campaign` — several launches against one rock (HANDOFF §8, Phase 3).
//!
//! The porkchop layer ([`crate::mission`]) answers *does one launch through this
//! window work*, and its honest headline is **no**: the best real launcher
//! delivers about 1/65th of what the headline curve wants. This module answers the
//! question that leaves: **how many launches, through which windows?**
//!
//! # Linear in the launches, by measurement not by assumption
//! A kinetic impactor's push is `β·(m/M)·v_rel` — linear in mass — and at the
//! millimetre-per-second scale of a real launch the b-plane responds linearly to
//! the push. So one full-field flight per window, at one launch's mass, gives that
//! window's **shift per launch**: a 2-D vector in the encounter's `(ξ, ζ)` plane.
//! Every campaign is then a sum of those vectors, and choosing one is arithmetic.
//!
//! That linearity is a *claim*, and this module does not rest on it: a chosen plan
//! is meant to be flown whole, chained through
//! [`DeflectionScenario::evaluate_campaign`](crate::deflection::DeflectionScenario::evaluate_campaign),
//! and the prediction compared with the flight. The planner here is only the
//! arithmetic half.
//!
//! # Direction matters: pushes can cancel
//! A window whose arrival meets the rock from behind speeds it up; one that meets
//! it head-on slows it down. On the b-plane those two move the aim point in
//! **opposite** directions, so a campaign that simply took the strongest windows
//! regardless of sign would spend launches undoing each other. The planner picks a
//! direction first and only uses windows that push along it.
//!
//! # What "fewest launches" means here
//! For a fixed direction `û`, taking the windows in order of their shift along
//! `û` — each up to what its period's cap has left — is the fewest launches that reach a given
//! distance **along `û`** (every launch adds a fixed amount, so the largest
//! amounts first is optimal). The planner tries each window's own direction and
//! its opposite, plus the nominal's, and stops a direction as soon as the true
//! `|B|` — not just its projection — clears the target. The cross-direction part
//! of `|B|` only ever helps, so this can finish *earlier* than the projection
//! alone says but never later. It is not a search over every mixture of
//! directions; on an encounter where the shifts are nearly collinear (the timing
//! axis `ζ̂` dominates — see the uncertainty-ellipse memory) the two coincide, and
//! the kernel-free tests pin optimality against brute force on collinear shifts.
//!
//! # The cap is per launch *period*, and it is a knob, not a fact
//! How many rockets can fly in a given stretch of time is set by pads, production
//! and cadence, and this project has no sourced number for it. So it is a
//! parameter with no default here. Anything that displays a launch count has to
//! display the cap it was counted under.
//!
//! The cap counts launches per **period** — a label each window carries — and not
//! per window. It was per window at first, and a window was one launch date on the
//! caller's grid, so the same cap allowed more launches per year on a finer grid:
//! the count measured the grid's resolution as much as the rocket. With periods of
//! a fixed length in time (the binding uses years), every window that falls in one
//! period draws on the same allowance, however many of them the grid happens to
//! offer. That keeps the planner exact: a per-period cap is a *partition*
//! constraint, and for a fixed direction taking the largest amounts first is still
//! the fewest launches (the brute-force test pins it with windows sharing periods).
//!
//! # And a rolling cap, exactly
//! "At most N in any 365 days" is the more physical rule, and it is not a partition
//! — a greedy fill can take one strong window that blocks two neighbours worth
//! more together. [`plan_campaign_rolling`] solves it exactly instead, by splitting
//! the launches into N chains each a period apart (see its doc), and is pinned
//! against brute force. The binding ships it; the per-period planner stays for
//! comparison. A fixed period is itself one of the rolling windows, so the
//! per-period count is a lower bound on the rolling one.
//!
//! # On mass: the impact mass, but no design margin — so not a floor
//! Each launch pushes with the mass that **arrives**, not the mass the rocket
//! lifts ([`crate::impactor_mass`]): DART's flown 3.6 % propellant loss comes off,
//! and nothing else does — the bus hits the rock, and the launch adapter is already
//! out of the tables. Measured on the 120×120 map's cells at every rate from 1 to
//! 10 a year, that moved the Falcon Heavy count only at 4 a year, from 6 to 7 (that
//! plan cleared the line by 1.1 %). 5 a year stayed 6, but cleared by only 0.4 %.
//! (Those windows were the map's cells; the continuous search since finds 6 at
//! every rate from 2 to 10 a year with the impact mass.)
//!
//! # Through a parking orbit, too
//! A launch can also go up to a parking orbit and leave later, on a better date,
//! with a departure stage of its own ([`crate::orbital_assembly`] prices the mass
//! that costs). [`parked_launches`] lists every such launch a plan could need and
//! [`parked_window`] turns one into a window the rolling planner takes like any
//! other — its launch date is what the cap counts, its departure window is where it
//! pushes — so [`plan_campaign_rolling`] stays exact over both ways of flying.
//!
//! What is still optimistic: the whole separated mass is assumed to be a buildable
//! impactor, with no design margin held back from the rocket's capability. But the
//! count is **not** a lower bound either: the planner only sees the windows its
//! caller flew, and a better window it was never shown would lower the count.
//! Optimistic on margin, and only as good as the caller's search — a count, not a
//! bound either way. (The binding's continuous window search is settled for the
//! push direction its plans use; its other direction is a lower bound.)

use nalgebra::{Vector2, Vector3};

use crate::epoch::Epoch;

/// One launch window as the planner sees it: what a single launch through it does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CampaignWindow {
    /// Launch epoch — carried for reporting and ordering, not used by the planner.
    pub launch_epoch: Epoch,
    /// Which launch period this window's launches count against. Windows sharing a
    /// period share one cap; the planner reads nothing else about time.
    pub period: u32,
    /// When the impactor reaches the rock, i.e. when the impulse is applied.
    pub arrival_epoch: Epoch,
    /// The impulse **one** launch imparts, m/s (SSB ICRF).
    pub impulse_per_launch: Vector3<f64>,
    /// Where one launch moves the b-plane aim point, `(ξ, ζ)` metres — measured
    /// by one full-field flight, relative to the nominal.
    pub shift_per_launch: Vector2<f64>,
}

/// A chosen campaign: how many launches through each window, and where the
/// arithmetic says they leave the aim point.
#[derive(Debug, Clone, PartialEq)]
pub struct CampaignPlan {
    /// Launches per window, parallel to the `windows` slice the plan was made from.
    pub launches: Vec<u32>,
    /// Sum of `launches`.
    pub total_launches: u32,
    /// Predicted aim point, `(ξ, ζ)` metres: nominal plus every launch's shift.
    pub predicted_b: Vector2<f64>,
}

impl CampaignPlan {
    /// `|predicted_b|`, metres — the predicted miss the hit test would read.
    pub fn predicted_impact_parameter(&self) -> f64 {
        self.predicted_b.norm()
    }
}

/// What the planner concluded.
#[derive(Debug, Clone, PartialEq)]
pub enum CampaignOutcome {
    /// The nominal already clears the target; no launch is needed.
    AlreadyClear,
    /// The fewest launches found that reach the target.
    Planned(CampaignPlan),
    /// No direction reaches the target even with every useful period at its cap.
    /// Carries the plan that got furthest, so a reader can see *how* short it is.
    Unreachable(CampaignPlan),
}

/// Why the planner refused its input.
#[derive(Debug, Clone, PartialEq)]
pub enum CampaignError {
    /// Non-finite or non-positive target, a zero cap, or a non-finite shift.
    InvalidInput(String),
}

impl std::fmt::Display for CampaignError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CampaignError::InvalidInput(m) => write!(f, "invalid campaign input: {m}"),
        }
    }
}

impl std::error::Error for CampaignError {}

/// Choose the fewest launches that move the aim point from `nominal_b` to at
/// least `target_b` from Earth's centre, with at most `max_launches_per_period`
/// launches across all the windows sharing a [`CampaignWindow::period`] (see the
/// module doc for exactly what "fewest" means and when it is exact).
///
/// `target_b` is an **impact parameter** — convert a perigee target with
/// [`OpikFrame::impact_parameter_for_perigee`](crate::keyhole::OpikFrame::impact_parameter_for_perigee)
/// first; a perigee and an impact parameter are not interchangeable.
pub fn plan_campaign(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    max_launches_per_period: u32,
) -> Result<CampaignOutcome, CampaignError> {
    validate(nominal_b, target_b, windows, max_launches_per_period)?;
    if nominal_b.norm() >= target_b {
        return Ok(CampaignOutcome::AlreadyClear);
    }
    Ok(best_over_directions(nominal_b, windows, |u| {
        fill_along(nominal_b, target_b, windows, max_launches_per_period, u)
    }))
}

/// The checks both planners make on their input.
fn validate(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    cap: u32,
) -> Result<(), CampaignError> {
    if !(target_b.is_finite() && target_b > 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "target impact parameter must be finite and > 0 (got {target_b})"
        )));
    }
    if cap == 0 {
        return Err(CampaignError::InvalidInput(
            "the launch cap must be at least 1".into(),
        ));
    }
    if !(nominal_b.x.is_finite() && nominal_b.y.is_finite()) {
        return Err(CampaignError::InvalidInput(format!(
            "nominal b-point is not finite ({nominal_b:?})"
        )));
    }
    for (i, w) in windows.iter().enumerate() {
        let finite = w.shift_per_launch.iter().all(|c| c.is_finite())
            && w.impulse_per_launch.iter().all(|c| c.is_finite());
        if !finite {
            return Err(CampaignError::InvalidInput(format!(
                "window {i} has a non-finite shift or impulse"
            )));
        }
    }
    Ok(())
}

/// Run `plan_along` over every candidate push direction and keep the best: the
/// fewest launches that reach (ties to the larger `|B|`), else the plan that got
/// furthest.
fn best_over_directions(
    nominal_b: Vector2<f64>,
    windows: &[CampaignWindow],
    mut plan_along: impl FnMut(Vector2<f64>) -> (CampaignPlan, bool),
) -> CampaignOutcome {
    // Candidate directions: each window's own and its opposite, and the nominal's
    // (a campaign that pushes the way the rock already misses starts ahead).
    let mut directions: Vec<Vector2<f64>> = Vec::with_capacity(2 * windows.len() + 1);
    for w in windows {
        let n = w.shift_per_launch.norm();
        if n > 0.0 {
            let u = w.shift_per_launch / n;
            directions.push(u);
            directions.push(-u);
        }
    }
    if nominal_b.norm() > 0.0 {
        directions.push(nominal_b.normalize());
    }
    // One direction once: a caller that scales one measured shift into many
    // estimated windows hands in thousands of copies of a few directions, and each
    // costs a full plan. Exact duplicates only (to rounding) — two directions that
    // differ at all can plan differently.
    let mut unique: Vec<Vector2<f64>> = Vec::with_capacity(directions.len());
    for u in directions {
        if !unique.iter().any(|q| q.dot(&u) > 1.0 - 1e-12) {
            unique.push(u);
        }
    }
    let directions = unique;

    let mut best_reached: Option<CampaignPlan> = None;
    let mut best_furthest: Option<CampaignPlan> = None;
    for u in directions {
        let (plan, reached) = plan_along(u);
        if reached {
            let better = match &best_reached {
                None => true,
                Some(b) => {
                    plan.total_launches < b.total_launches
                        || (plan.total_launches == b.total_launches
                            && plan.predicted_impact_parameter() > b.predicted_impact_parameter())
                }
            };
            if better {
                best_reached = Some(plan);
            }
        } else {
            let better = best_furthest
                .as_ref()
                .is_none_or(|b| plan.predicted_impact_parameter() > b.predicted_impact_parameter());
            if better {
                best_furthest = Some(plan);
            }
        }
    }

    match (best_reached, best_furthest) {
        (Some(p), _) => CampaignOutcome::Planned(p),
        (None, Some(p)) => CampaignOutcome::Unreachable(p),
        // No window moves the aim point at all.
        (None, None) => CampaignOutcome::Unreachable(CampaignPlan {
            launches: vec![0; windows.len()],
            total_launches: 0,
            predicted_b: nominal_b,
        }),
    }
}

/// How far short of a whole rolling window two launches may be and still count as
/// a window apart, s. Dates built as `t + k·P` land a whole number of periods apart
/// only to rounding (~1e-7 s at these epochs), so both the cap count and the
/// planner read "a period apart" with this slack — the same slack, so the two can
/// never disagree. A millisecond is nothing to a launch rule measured in days.
pub const ROLLING_SLACK_S: f64 = 1.0e-3;

/// The most launches inside any one rolling window of `window_s` seconds,
/// counting each window as half-open `[t, t + window_s)` and starting one at
/// every launch — from `(launch epoch as TDB seconds past J2000, launches)` pairs.
/// Launches within [`ROLLING_SLACK_S`] of a window's far end count as outside it.
pub fn busiest_rolling_count(launches: &[(f64, u32)], window_s: f64) -> u32 {
    launches
        .iter()
        .map(|&(t0, _)| {
            launches
                .iter()
                .filter(|&&(t, _)| t >= t0 && t < t0 + window_s - ROLLING_SLACK_S)
                .map(|&(_, n)| n)
                .sum()
        })
        .max()
        .unwrap_or(0)
}

/// Choose the fewest launches that move the aim point from `nominal_b` to at
/// least `target_b`, with **at most `max_launches` inside any `window_s` seconds**
/// — a rolling cap, counted exactly as [`busiest_rolling_count`] counts it. Each
/// window's [`CampaignWindow::period`] is ignored; its launch epoch is what counts.
///
/// # Exact, by turning the cap into chains
/// "At most `N` in any window of length `P`" is the same as "the launches split
/// into `N` chains, each with consecutive launches at least `P` apart":
/// - chains → cap: a half-open window of length `P` holds at most one launch of a
///   chain, so at most `N` in all;
/// - cap → chains: sort the launches and deal launch `i` to chain `i mod N`. The
///   window `[tᵢ, tᵢ + P)` holds at most `N` launches, and `tᵢ … tᵢ₊ₙ₋₁` are already
///   `N` of them, so `tᵢ₊ₙ ≥ tᵢ + P`.
///
/// The chains do not interact (two may even launch on the same date — that is
/// two rockets at once, which the cap allows). So along a direction `û`, the best
/// `L` launches are the best split of `L` across `N` copies of one problem — "the
/// best chain of `m` launches at least `P` apart" — which is a small dynamic
/// programme over the windows in date order. No greedy step: the fill the
/// fixed-period planner uses is wrong here, because one strong window can block
/// two neighbours worth more together (a brute-force test pins that case).
///
/// Directions, and the `|B|` stopping test, are exactly [`plan_campaign`]'s: for
/// each `L` the arrangement with the largest shift along `û` is checked against
/// the target with its true `|B|`.
pub fn plan_campaign_rolling(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    max_launches: u32,
    window_s: f64,
) -> Result<CampaignOutcome, CampaignError> {
    validate(nominal_b, target_b, windows, max_launches)?;
    if !(window_s.is_finite() && window_s > 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "the rolling window must be finite and > 0 s (got {window_s})"
        )));
    }
    if nominal_b.norm() >= target_b {
        return Ok(CampaignOutcome::AlreadyClear);
    }
    Ok(best_over_directions(nominal_b, windows, |u| {
        chains_along(nominal_b, target_b, windows, max_launches, window_s, u)
    }))
}

/// [`plan_campaign_rolling`] along one direction: the fewest launches whose best
/// arrangement under the rolling cap clears `target_b`, else the furthest.
fn chains_along(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    cap: u32,
    window_s: f64,
    u: Vector2<f64>,
) -> (CampaignPlan, bool) {
    let empty = CampaignPlan {
        launches: vec![0; windows.len()],
        total_launches: 0,
        predicted_b: nominal_b,
    };
    // The useful windows in date order, with their value along `u`.
    let mut idx: Vec<usize> = (0..windows.len())
        .filter(|&i| windows[i].shift_per_launch.dot(&u) > 0.0)
        .collect();
    let when = |i: usize| windows[i].launch_epoch.tdb_seconds_past_j2000();
    idx.sort_by(|&a, &b| when(a).total_cmp(&when(b)));
    let w = idx.len();
    if w == 0 {
        return (empty, false);
    }
    let t: Vec<f64> = idx.iter().map(|&i| when(i)).collect();
    let v: Vec<f64> = idx
        .iter()
        .map(|&i| windows[i].shift_per_launch.dot(&u))
        .collect();
    // back[k]: every window before this position launches at least `window_s`
    // before window k (to `ROLLING_SLACK_S`), so a chain can step from it to k.
    let back: Vec<usize> = (0..w)
        .map(|k| t.partition_point(|&tj| tj <= t[k] - window_s + ROLLING_SLACK_S))
        .collect();

    // One chain. best[m-1][k]: the most value an m-launch chain ending at window k
    // carries (-inf if none fits); from[m-1][k]: the window it steps back to;
    // prefix[m-1][k]: (value, window) of the best m-chain ending at or before k.
    let prefix_of = |row: &[f64]| -> Vec<(f64, usize)> {
        let mut acc = (f64::NEG_INFINITY, 0);
        row.iter()
            .enumerate()
            .map(|(k, &x)| {
                if x > acc.0 {
                    acc = (x, k);
                }
                acc
            })
            .collect()
    };
    let mut from: Vec<Vec<Option<usize>>> = vec![vec![None; w]];
    let mut prefix = vec![prefix_of(&v)];
    loop {
        let prev = prefix.last().expect("one row at least");
        let (row, link): (Vec<f64>, Vec<Option<usize>>) = (0..w)
            .map(|k| match back[k] {
                0 => (f64::NEG_INFINITY, None),
                b => {
                    let (val, j) = prev[b - 1];
                    if val.is_finite() {
                        (val + v[k], Some(j))
                    } else {
                        (f64::NEG_INFINITY, None)
                    }
                }
            })
            .unzip();
        if !row.iter().any(|x| x.is_finite()) {
            break;
        }
        prefix.push(prefix_of(&row));
        from.push(link);
    }
    let longest = prefix.len(); // the most launches one chain can hold
    let value = |m: usize| if m == 0 { 0.0 } else { prefix[m - 1][w - 1].0 };
    // The windows (positions in `idx`) of the best m-launch chain.
    let chain = |m: usize| -> Vec<usize> {
        let mut k = prefix[m - 1][w - 1].1;
        let mut out = vec![k];
        for level in (1..m).rev() {
            k = from[level][k].expect("a chain of more than one launch steps back");
            out.push(k);
        }
        out
    };

    // L launches split across `cap` identical chains: split[c][l] is the most value
    // c chains carry with l launches between them, and how many the last one took.
    let cap = cap as usize;
    let max_l = cap * longest;
    let mut split = vec![vec![(f64::NEG_INFINITY, 0usize); max_l + 1]; cap + 1];
    split[0][0] = (0.0, 0);
    for c in 1..=cap {
        for l in 0..=max_l {
            for m in 0..=l.min(longest) {
                let rest = split[c - 1][l - m].0;
                if rest.is_finite() && rest + value(m) > split[c][l].0 {
                    split[c][l] = (rest + value(m), m);
                }
            }
        }
    }
    let plan_for = |l: usize| -> CampaignPlan {
        let mut launches = vec![0u32; windows.len()];
        let mut l = l;
        for c in (1..=cap).rev() {
            let m = split[c][l].1;
            if m > 0 {
                for k in chain(m) {
                    launches[idx[k]] += 1;
                }
            }
            l -= m;
        }
        let predicted_b = launches.iter().zip(windows).fold(nominal_b, |b, (&n, w)| {
            b + f64::from(n) * w.shift_per_launch
        });
        CampaignPlan {
            total_launches: launches.iter().sum(),
            launches,
            predicted_b,
        }
    };
    // Short of the target, the plan that gets furthest — which need not be the one
    // with the most launches: under the cap, one more launch can force the whole
    // arrangement onto weaker dates (three weak dates can sum to less than two
    // strong ones a period apart).
    let mut furthest = empty;
    for (l, &(reach, _)) in split[cap].iter().enumerate().skip(1) {
        if !reach.is_finite() {
            continue;
        }
        let p = plan_for(l);
        if p.predicted_impact_parameter() >= target_b {
            return (p, true);
        }
        if p.predicted_impact_parameter() > furthest.predicted_impact_parameter() {
            furthest = p;
        }
    }
    (furthest, false)
}

// --- A limited stock ---------------------------------------------------------

/// How a limited stock of ready interceptors sits beside the yearly cap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StockMode {
    /// Stocked launches count against the same "N in any 12 months" as every other
    /// launch: the stock only changes *when* the first ones can go. The shipping mode.
    Counted,
    /// Stocked launches fly outside the cap. Since the cap already allows several
    /// launches on one date, with no cap of their own they all go through the single
    /// best window on or after the stock's date, on one day - an optimistic what-if
    /// (it assumes as many pads as stocked rockets), not a surge rate anyone sourced.
    OutsideCap,
}

/// A limited stock: `size` interceptors ready from the windows' own first date, and
/// every other launch no earlier than `build_from` (the build-from-scratch date) -
/// built ones, which come `built_per_period` at a time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stock {
    /// How many launches can go before `build_from`. `u32::MAX` is "every launch".
    pub size: u32,
    /// The first date a launch that is not from the stock can go.
    pub build_from: Epoch,
    /// Counted against the cap, or outside it.
    pub mode: StockMode,
    /// How many built impactors a production line finishes per period: the first
    /// lot at `build_from`, another lot each period after. A finished impactor waits
    /// for its launch, so lots add up. [`UNLIMITED_BUILDS`] is no limit; any value at
    /// or above the launch cap is the same (see [`plan_campaign_stocked`]).
    pub built_per_period: u32,
}

/// [`Stock::built_per_period`] with no limit: every built launch is ready from
/// `build_from` on, whenever the cap allows.
pub const UNLIMITED_BUILDS: u32 = u32::MAX;

/// [`plan_campaign_rolling`] with a limited stock: launches dated before
/// `stock.build_from` must come from the stock, and at most `stock.size` of them
/// can ([`StockMode::Counted`]); or the stock flies outside the cap, all through
/// the best window ([`StockMode::OutsideCap`]). Windows are offered as they are -
/// the caller has already cut them at the stock's own first date.
///
/// # Exact, by the same chains
/// Counted, the rolling cap still splits the launches into `cap` chains a period
/// apart ([`plan_campaign_rolling`]); the stock adds one number per chain - how many
/// of its launches are before `build_from` - and the chains' early launches must
/// sum to at most `size`. Each chain's best value is tabulated by (launches, early
/// launches), and the chains are combined over both. A stocked launch dated after
/// `build_from` is indistinguishable from a built one, so "at most `size` early" is
/// the whole constraint. With `size` large enough it is no constraint and the plan
/// is [`plan_campaign_rolling`]'s; with `size` 0 it is that planner on the windows
/// from `build_from` on (both pinned by tests, with a brute force where the stock
/// binds).
///
/// Outside the cap, `k <= size` stocked launches through the best window (along the
/// direction) are added to the best built-only plan of the remaining launches, and
/// each total is checked with the arrangement that pushes furthest along it.
///
/// # A production line, exactly, by chains with release dates
/// With `built_per_period` below the cap, built impactors arrive in lots: `B` at
/// `build_from`, `B` more each period after, and a finished one can wait for a
/// better date - so before the `k+1`-th lot arrives at most `size + (k+1)·B`
/// launches can have gone. That is a release date on each launch's *rank* in date
/// order, and dealt to chains the ranks stay put, so the chains are independent
/// again - `cap` different ones instead of `cap` copies of one (see `LotChains`).
/// Exact, and pinned three ways: against brute force, against an independent exact
/// solver ([`crate::interval_packing`], which reads every rule as a cap on a run of
/// windows in date order), and at `B` = the cap against the chains above. A lot is
/// counted as arriving `k·ROLLING_SLACK_S` early, the slack the rolling cap reads "a
/// period apart" with, so that at `B` = the cap the lots are provably no constraint
/// - and below it the chain planners above are not used.
pub fn plan_campaign_stocked(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    max_launches: u32,
    window_s: f64,
    stock: &Stock,
) -> Result<CampaignOutcome, CampaignError> {
    validate(nominal_b, target_b, windows, max_launches)?;
    if !(window_s.is_finite() && window_s > 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "the rolling window must be finite and > 0 s (got {window_s})"
        )));
    }
    if nominal_b.norm() >= target_b {
        return Ok(CampaignOutcome::AlreadyClear);
    }
    let build_from = stock.build_from.tdb_seconds_past_j2000();
    // A production line slower than the cap: the lots bind, and only the exact
    // packing carries them. (A stock of every launch never needs a built one.)
    if stock.built_per_period < max_launches && stock.size != u32::MAX {
        return Ok(plan_with_lots(
            nominal_b,
            target_b,
            windows,
            max_launches,
            window_s,
            stock,
        ));
    }
    // A stock no plan can exhaust is no constraint: the rolling planner, exactly.
    // One chain holds at most one launch per period before `build_from`, so `cap`
    // chains hold at most `cap` per period of the early span.
    if stock.mode == StockMode::Counted {
        let early: Vec<f64> = windows
            .iter()
            .map(|w| w.launch_epoch.tdb_seconds_past_j2000())
            .filter(|&t| t < build_from)
            .collect();
        let slots = match (
            early.iter().copied().reduce(f64::min),
            early.iter().copied().reduce(f64::max),
        ) {
            (Some(lo), Some(hi)) => ((hi - lo) / window_s).floor() as u64 + 1,
            _ => 0,
        };
        if u64::from(stock.size) >= u64::from(max_launches) * slots {
            return plan_campaign_rolling(nominal_b, target_b, windows, max_launches, window_s);
        }
    }
    Ok(best_over_directions(nominal_b, windows, |u| {
        match stock.mode {
            StockMode::Counted => stocked_chains_along(
                nominal_b,
                target_b,
                windows,
                max_launches,
                window_s,
                u,
                build_from,
                stock.size,
            ),
            StockMode::OutsideCap => outside_cap_along(
                nominal_b,
                target_b,
                windows,
                max_launches,
                window_s,
                u,
                stock,
            ),
        }
    }))
}

/// Per-chain tables and their combination across `cap` chains: the best value
/// along `u` of any arrangement of `l` launches with `e` of them before
/// `build_from`, under the rolling cap. `stock` bounds the early launches in total.
struct StockedChains {
    /// Useful windows (indices into the caller's slice), in date order.
    idx: Vec<usize>,
    /// `from[m-1][e][k]`: the window an `m`-launch, `e`-early chain ending at `k`
    /// steps back to.
    from: Vec<Vec<Vec<Option<usize>>>>,
    /// `prefix[m-1][e][k]`: (value, window) of the best such chain ending at or before `k`.
    prefix: Vec<Vec<Vec<(f64, usize)>>>,
    /// `early[k]`: window `k` (in date order) is before `build_from`.
    early: Vec<bool>,
    /// `split[c][l][e]`: best value of `c` chains with `l` launches, `e` early, and
    /// the (launches, early) the last chain took.
    split: Vec<Vec<Vec<(f64, usize, usize)>>>,
    cap: usize,
    e_max: usize,
}

impl StockedChains {
    fn build(
        windows: &[CampaignWindow],
        cap: u32,
        window_s: f64,
        u: Vector2<f64>,
        build_from: f64,
        stock: u32,
    ) -> Option<Self> {
        let mut idx: Vec<usize> = (0..windows.len())
            .filter(|&i| windows[i].shift_per_launch.dot(&u) > 0.0)
            .collect();
        let when = |i: usize| windows[i].launch_epoch.tdb_seconds_past_j2000();
        idx.sort_by(|&a, &b| when(a).total_cmp(&when(b)));
        let w = idx.len();
        if w == 0 {
            return None;
        }
        let t: Vec<f64> = idx.iter().map(|&i| when(i)).collect();
        let v: Vec<f64> = idx
            .iter()
            .map(|&i| windows[i].shift_per_launch.dot(&u))
            .collect();
        let early: Vec<bool> = t.iter().map(|&tk| tk < build_from).collect();
        let back: Vec<usize> = (0..w)
            .map(|k| t.partition_point(|&tj| tj <= t[k] - window_s + ROLLING_SLACK_S))
            .collect();
        // One chain holds at most one launch per period, so at most this many early
        // ones - which keeps the early dimension small even for an unbounded stock.
        let early_slots = match (t.first(), early.iter().rposition(|&e| e)) {
            (Some(&t0), Some(last)) => ((t[last] - t0) / window_s).floor() as usize + 1,
            _ => 0,
        };
        let cap = cap as usize;
        let e_chain = early_slots.min(stock as usize);
        let e_max = (cap * early_slots).min(stock as usize);

        let ninf = (f64::NEG_INFINITY, 0usize);
        let prefix_of = |row: &[f64]| -> Vec<(f64, usize)> {
            let mut acc = ninf;
            row.iter()
                .enumerate()
                .map(|(k, &x)| {
                    if x > acc.0 {
                        acc = (x, k);
                    }
                    acc
                })
                .collect()
        };
        // m = 1.
        let first: Vec<Vec<f64>> = (0..=e_chain)
            .map(|e| {
                (0..w)
                    .map(|k| {
                        if usize::from(early[k]) == e {
                            v[k]
                        } else {
                            f64::NEG_INFINITY
                        }
                    })
                    .collect()
            })
            .collect();
        let mut from: Vec<Vec<Vec<Option<usize>>>> = vec![vec![vec![None; w]; e_chain + 1]];
        let mut prefix: Vec<Vec<Vec<(f64, usize)>>> =
            vec![first.iter().map(|r| prefix_of(r)).collect()];
        loop {
            let prev = prefix.last().expect("one row at least");
            let mut rows = Vec::with_capacity(e_chain + 1);
            let mut links = Vec::with_capacity(e_chain + 1);
            let mut any = false;
            for e in 0..=e_chain {
                let (row, link): (Vec<f64>, Vec<Option<usize>>) = (0..w)
                    .map(|k| {
                        let ek = usize::from(early[k]);
                        if back[k] == 0 || e < ek {
                            return (f64::NEG_INFINITY, None);
                        }
                        let (val, j) = prev[e - ek][back[k] - 1];
                        if val.is_finite() {
                            (val + v[k], Some(j))
                        } else {
                            (f64::NEG_INFINITY, None)
                        }
                    })
                    .unzip();
                any |= row.iter().any(|x| x.is_finite());
                rows.push(prefix_of(&row));
                links.push(link);
            }
            if !any {
                break;
            }
            prefix.push(rows);
            from.push(links);
        }
        let longest = prefix.len();
        let value = |m: usize, e: usize| -> f64 {
            if m == 0 {
                if e == 0 {
                    0.0
                } else {
                    f64::NEG_INFINITY
                }
            } else if e > e_chain {
                f64::NEG_INFINITY
            } else {
                prefix[m - 1][e][w - 1].0
            }
        };
        let max_l = cap * longest;
        let mut split =
            vec![vec![vec![(f64::NEG_INFINITY, 0usize, 0usize); e_max + 1]; max_l + 1]; cap + 1];
        split[0][0][0] = (0.0, 0, 0);
        for c in 1..=cap {
            for l in 0..=max_l {
                for e_tot in 0..=e_max {
                    for m in 0..=l.min(longest) {
                        for e in 0..=e_tot.min(m).min(e_chain) {
                            let rest = split[c - 1][l - m][e_tot - e].0;
                            let here = value(m, e);
                            if rest.is_finite()
                                && here.is_finite()
                                && rest + here > split[c][l][e_tot].0
                            {
                                split[c][l][e_tot] = (rest + here, m, e);
                            }
                        }
                    }
                }
            }
        }
        Some(Self {
            idx,
            from,
            prefix,
            early,
            split,
            cap,
            e_max,
        })
    }

    /// The most launches any arrangement can hold.
    fn max_launches(&self) -> usize {
        self.split[self.cap].len() - 1
    }

    /// The best value of `l` launches and the early count it takes, if any arrangement fits.
    fn best(&self, l: usize) -> Option<(f64, usize)> {
        let row = self.split[self.cap].get(l)?;
        let (e, &(val, _, _)) = row
            .iter()
            .enumerate()
            .take(self.e_max + 1)
            .max_by(|a, b| a.1 .0.total_cmp(&b.1 .0))?;
        val.is_finite().then_some((val, e))
    }

    /// Launches per caller window for the best `l`-launch arrangement.
    fn launches(&self, n_windows: usize, l: usize) -> Vec<u32> {
        let mut launches = vec![0u32; n_windows];
        let Some((_, mut e_tot)) = self.best(l) else {
            return launches;
        };
        let mut l = l;
        let w = self.idx.len();
        for c in (1..=self.cap).rev() {
            let (_, m, e) = self.split[c][l][e_tot];
            if m > 0 {
                let mut k = self.prefix[m - 1][e][w - 1].1;
                let mut e_left = e;
                launches[self.idx[k]] += 1;
                for level in (1..m).rev() {
                    let step = self.from[level][e_left][k]
                        .expect("a chain of more than one launch steps back");
                    e_left -= usize::from(self.early[k]);
                    k = step;
                    launches[self.idx[k]] += 1;
                }
            }
            l -= m;
            e_tot -= e;
        }
        launches
    }
}

/// The plan for `launches`, with its predicted aim point.
fn plan_of(
    nominal_b: Vector2<f64>,
    windows: &[CampaignWindow],
    launches: Vec<u32>,
) -> CampaignPlan {
    let predicted_b = launches.iter().zip(windows).fold(nominal_b, |b, (&n, w)| {
        b + f64::from(n) * w.shift_per_launch
    });
    CampaignPlan {
        total_launches: launches.iter().sum(),
        launches,
        predicted_b,
    }
}

/// [`plan_campaign_stocked`], counted, along one direction.
#[allow(clippy::too_many_arguments)]
fn stocked_chains_along(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    cap: u32,
    window_s: f64,
    u: Vector2<f64>,
    build_from: f64,
    stock: u32,
) -> (CampaignPlan, bool) {
    let empty = plan_of(nominal_b, windows, vec![0; windows.len()]);
    let Some(t) = StockedChains::build(windows, cap, window_s, u, build_from, stock) else {
        return (empty, false);
    };
    let mut furthest = empty;
    for l in 1..=t.max_launches() {
        if t.best(l).is_none() {
            continue;
        }
        let p = plan_of(nominal_b, windows, t.launches(windows.len(), l));
        if p.predicted_impact_parameter() >= target_b {
            return (p, true);
        }
        if p.predicted_impact_parameter() > furthest.predicted_impact_parameter() {
            furthest = p;
        }
    }
    (furthest, false)
}

/// [`plan_campaign_stocked`], outside the cap, along one direction.
fn outside_cap_along(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    cap: u32,
    window_s: f64,
    u: Vector2<f64>,
    stock: &Stock,
) -> (CampaignPlan, bool) {
    let empty = plan_of(nominal_b, windows, vec![0; windows.len()]);
    // The stock's window: the best along `u` at any date.
    let best = (0..windows.len())
        .map(|i| (i, windows[i].shift_per_launch.dot(&u)))
        .filter(|&(_, x)| x > 0.0)
        .max_by(|a, b| a.1.total_cmp(&b.1));
    // The built launches: no stock at all, so nothing before `build_from`.
    let build_from = stock.build_from.tdb_seconds_past_j2000();
    let built = StockedChains::build(windows, cap, window_s, u, build_from, 0);
    let built_max = built.as_ref().map_or(0, |b| b.max_launches());
    // An unbounded stock still stops: every stocked launch adds the same push along
    // `u`, so the projection - a lower bound on `|B|` - passes any target; the clamp
    // only guards a vanishing push.
    let stock_max = if best.is_some() {
        (stock.size as usize).min(10_000)
    } else {
        0
    };
    let mut furthest = empty;
    for l in 1..=stock_max.saturating_add(built_max) {
        // The split of `l` that pushes furthest along `u`.
        let mut top: Option<(f64, usize)> = None;
        for k in 0..=l.min(stock_max) {
            let rest = l - k;
            let built_val = if rest == 0 {
                Some(0.0)
            } else {
                built.as_ref().and_then(|b| b.best(rest)).map(|x| x.0)
            };
            let Some(bv) = built_val else { continue };
            let val = bv + k as f64 * best.map_or(0.0, |x| x.1);
            if top.is_none_or(|(tv, _)| val > tv) {
                top = Some((val, k));
            }
        }
        let Some((_, k)) = top else { continue };
        let mut launches = match (&built, l - k) {
            (_, 0) => vec![0; windows.len()],
            (Some(b), rest) => b.launches(windows.len(), rest),
            (None, _) => continue,
        };
        if let Some((i, _)) = best {
            launches[i] += k as u32;
        }
        let p = plan_of(nominal_b, windows, launches);
        if p.predicted_impact_parameter() >= target_b {
            return (p, true);
        }
        if p.predicted_impact_parameter() > furthest.predicted_impact_parameter() {
            furthest = p;
        }
    }
    (furthest, false)
}

// --- A stock already in orbit -------------------------------------------------

/// A plan with `size` stacks **already in orbit** when the rock is found
/// ([`crate::station_keeping::StandingStack`]) beside the launched ones: the stacks
/// leave through `orbit` - their own windows, each `orbit[j]` a departure with the
/// push one stack makes through it - and every launch goes through `windows` under
/// the cap, built no earlier than `built.build_from` and `built.built_per_period` a
/// year (`built.size` and `built.mode` are ignored: the launched ones have no stock
/// of their own).
///
/// The stacks are not launches - they went up years ago - so the cap does not count
/// them, and any number can leave on one date. So, as for
/// [`StockMode::OutsideCap`], `k <= size` of them all leave through the best orbit
/// window along the push direction, and the rest is the best built-only plan; each
/// total is checked with the split that pushes furthest. (Outside the cap is a
/// what-if for a stock on the ground - it would need as many pads as rockets. For a
/// stack in orbit it is simply true.)
///
/// The returned `launches` run over `windows` then `orbit`, in that order.
#[allow(clippy::too_many_arguments)]
pub fn plan_campaign_orbit_stocked(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    orbit: &[CampaignWindow],
    max_launches: u32,
    window_s: f64,
    size: u32,
    built: &Stock,
) -> Result<CampaignOutcome, CampaignError> {
    let all: Vec<CampaignWindow> = windows.iter().chain(orbit).copied().collect();
    validate(nominal_b, target_b, &all, max_launches)?;
    if !(window_s.is_finite() && window_s > 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "the rolling window must be finite and > 0 s (got {window_s})"
        )));
    }
    if nominal_b.norm() >= target_b {
        return Ok(CampaignOutcome::AlreadyClear);
    }
    let n = windows.len();
    let built = Stock {
        size: 0,
        mode: StockMode::OutsideCap,
        ..*built
    };
    let build_from = built.build_from.tdb_seconds_past_j2000();
    let lots = built.built_per_period < max_launches;
    Ok(best_over_directions(nominal_b, &all, |u| {
        let best = (0..orbit.len())
            .map(|j| (n + j, orbit[j].shift_per_launch.dot(&u)))
            .filter(|&(_, x)| x > 0.0)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        let stock_max = if best.is_some() {
            (size as usize).min(10_000)
        } else {
            0
        };
        // The built launches' best `l`-launch arrangement, over `windows` only.
        let chains = if lots {
            None
        } else {
            StockedChains::build(windows, max_launches, window_s, u, build_from, 0)
        };
        let line = lots.then(|| LotChains::build(windows, max_launches, window_s, u, &built));
        let built_best = |l: usize| -> Option<(f64, Vec<u32>)> {
            if l == 0 {
                return Some((0.0, vec![0; n]));
            }
            match (&chains, &line) {
                (Some(c), _) => c.best(l).map(|(v, _)| (v, c.launches(n, l))),
                (None, Some(lc)) => lc.best(l, n),
                (None, None) => None,
            }
        };
        let mut furthest = plan_of(nominal_b, &all, vec![0; all.len()]);
        for l in 1usize.. {
            let mut top: Option<(f64, usize, Vec<u32>)> = None;
            for k in 0..=l.min(stock_max) {
                let Some((bv, launches)) = built_best(l - k) else {
                    continue;
                };
                let val = bv + k as f64 * best.map_or(0.0, |x| x.1);
                if top.as_ref().is_none_or(|(tv, _, _)| val > *tv) {
                    top = Some((val, k, launches));
                }
            }
            // Nothing fits at `l`: no larger `l` does either.
            let Some((_, k, mut launches)) = top else {
                break;
            };
            launches.resize(all.len(), 0);
            if let Some((i, _)) = best {
                launches[i] += k as u32;
            }
            let p = plan_of(nominal_b, &all, launches);
            if p.predicted_impact_parameter() >= target_b {
                return (p, true);
            }
            if p.predicted_impact_parameter() > furthest.predicted_impact_parameter() {
                furthest = p;
            }
        }
        (furthest, false)
    }))
}

// --- A production line --------------------------------------------------------

/// [`plan_campaign_stocked`] with built impactors arriving in lots - for any
/// `built_per_period`, though the public planner only sends it here below the cap
/// (at or above it the lots are no constraint, and the tests pin that this route
/// then agrees with the chains it would otherwise take).
fn plan_with_lots(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    cap: u32,
    window_s: f64,
    stock: &Stock,
) -> CampaignOutcome {
    best_over_directions(nominal_b, windows, |u| {
        let lots = LotChains::build(windows, cap, window_s, u, stock);
        match stock.mode {
            StockMode::Counted => counted_lots_along(nominal_b, target_b, windows, lots),
            StockMode::OutsideCap => {
                outside_cap_lots_along(nominal_b, target_b, windows, lots, u, stock)
            }
        }
    })
}

/// The first date the `rank`-th launch (1-based, in date order) can go: the stock's
/// `stocked` from any date, then lot `j` (from 0) of `per` built impactors from
/// `build_from + j·step`. With no production at all, never.
fn lot_release(rank: usize, stocked: u64, per: u64, build_from: f64, step: f64) -> f64 {
    let rank = rank as u64;
    if rank <= stocked {
        return f64::NEG_INFINITY;
    }
    match (rank - stocked - 1).checked_div(per) {
        Some(lot) => build_from + lot as f64 * step,
        None => f64::INFINITY,
    }
}

/// One chain's table: `prefix[m-1][k]` and `from[m-1][k]` (see [`LotChains`]).
type ChainTable = (Vec<Vec<(f64, usize)>>, Vec<Vec<Option<usize>>>);

/// Exact plans under the rolling cap, the stock and a production line, by chains
/// again - `cap` of them, no longer identical.
///
/// # Why chains still work
/// Sort a plan's launches by date. The lots say: before lot `k+1` arrives at most
/// `stocked + (k+1)·B` have gone - which is the same as "the `n`-th launch goes no
/// earlier than the lot that finishes impactor `n`" ([`lot_release`]), a release
/// date that depends only on the launch's *rank*. Deal the sorted launches to `cap`
/// chains in turn, as [`plan_campaign_rolling`] does: each chain's launches are a
/// period apart, and chain `r`'s `m`-th launch is rank `r + 1 + (m−1)·cap` - so it
/// carries that rank's release. Conversely, chains that each respect their own
/// releases always merge into a plan that respects the lots, whatever their sizes:
/// before any lot date, chain `r` can only have launches whose ranks are within that
/// lot's total, and the ranks the chains own are disjoint, so together they hold at
/// most that total. So the chains are independent again; they just no longer share
/// one table, since chain `r` sees the releases of ranks `r+1, r+1+cap, …`.
struct LotChains {
    /// Useful windows (indices into the caller's slice), in date order.
    idx: Vec<usize>,
    /// Per chain: `prefix[m-1][k]`, (value, window) of the best `m`-launch chain
    /// ending at or before window `k`; and `from[m-1][k]`, the window it steps back
    /// to.
    chains: Vec<ChainTable>,
    /// `split[c][l]`: best value of the first `c` chains holding `l` launches, and
    /// how many the `c`-th took.
    split: Vec<Vec<(f64, usize)>>,
}

impl LotChains {
    /// Counted: the stock's `size` are the first ranks. Outside the cap: the built
    /// launches only, none before `build_from`.
    fn build(
        windows: &[CampaignWindow],
        cap: u32,
        window_s: f64,
        u: Vector2<f64>,
        stock: &Stock,
    ) -> Self {
        let when = |i: usize| windows[i].launch_epoch.tdb_seconds_past_j2000();
        let mut idx: Vec<usize> = (0..windows.len())
            .filter(|&i| windows[i].shift_per_launch.dot(&u) > 0.0)
            .collect();
        idx.sort_by(|&a, &b| when(a).total_cmp(&when(b)));
        let t: Vec<f64> = idx.iter().map(|&i| when(i)).collect();
        let v: Vec<f64> = idx
            .iter()
            .map(|&i| windows[i].shift_per_launch.dot(&u))
            .collect();
        let w = idx.len();
        let back: Vec<usize> = (0..w)
            .map(|k| t.partition_point(|&tj| tj <= t[k] - window_s + ROLLING_SLACK_S))
            .collect();
        let stocked = match stock.mode {
            StockMode::Counted => u64::from(stock.size),
            StockMode::OutsideCap => 0,
        };
        let per = u64::from(stock.built_per_period);
        let build_from = stock.build_from.tdb_seconds_past_j2000();
        // Lot `j` arrives `j` slacks early, so that at `B` = the cap the lots are
        // provably no constraint: launches a period apart to the rolling cap's slack
        // are a period apart here too.
        let step = window_s - ROLLING_SLACK_S;
        let release = |rank: usize| lot_release(rank, stocked, per, build_from, step);
        let prefix_of = |row: &[f64]| -> Vec<(f64, usize)> {
            let mut acc = (f64::NEG_INFINITY, 0);
            row.iter()
                .enumerate()
                .map(|(k, &x)| {
                    if x > acc.0 {
                        acc = (x, k);
                    }
                    acc
                })
                .collect()
        };
        let cap = cap as usize;
        let mut chains = Vec::with_capacity(cap);
        for r in 0..cap {
            let mut prefix: Vec<Vec<(f64, usize)>> = Vec::new();
            let mut from: Vec<Vec<Option<usize>>> = Vec::new();
            for m in 1usize.. {
                let not_before = release(r + 1 + (m - 1) * cap);
                let (row, link): (Vec<f64>, Vec<Option<usize>>) = (0..w)
                    .map(|k| {
                        if t[k] < not_before {
                            return (f64::NEG_INFINITY, None);
                        }
                        if m == 1 {
                            return (v[k], None);
                        }
                        if back[k] == 0 {
                            return (f64::NEG_INFINITY, None);
                        }
                        let (val, j) = prefix[m - 2][back[k] - 1];
                        if val.is_finite() {
                            (val + v[k], Some(j))
                        } else {
                            (f64::NEG_INFINITY, None)
                        }
                    })
                    .unzip();
                if !row.iter().any(|x| x.is_finite()) {
                    break;
                }
                prefix.push(prefix_of(&row));
                from.push(link);
            }
            chains.push((prefix, from));
        }
        // Combine the chains, each with its own table.
        let value = |c: usize, m: usize| -> f64 {
            let prefix = &chains[c].0;
            if m == 0 {
                0.0
            } else if m > prefix.len() || w == 0 {
                f64::NEG_INFINITY
            } else {
                prefix[m - 1][w - 1].0
            }
        };
        let max_l: usize = chains.iter().map(|ch| ch.0.len()).sum();
        let mut split = vec![vec![(f64::NEG_INFINITY, 0usize); max_l + 1]; cap + 1];
        split[0][0] = (0.0, 0);
        for c in 1..=cap {
            let longest = chains[c - 1].0.len();
            for l in 0..=max_l {
                for m in 0..=l.min(longest) {
                    let rest = split[c - 1][l - m].0;
                    let here = value(c - 1, m);
                    if rest.is_finite() && here.is_finite() && rest + here > split[c][l].0 {
                        split[c][l] = (rest + here, m);
                    }
                }
            }
        }
        Self { idx, chains, split }
    }

    /// The best `l`-launch arrangement, as launches per caller window, and its value
    /// along the direction - `None` where `l` launches do not fit.
    fn best(&self, l: usize, n_windows: usize) -> Option<(f64, Vec<u32>)> {
        let cap = self.chains.len();
        let (value, _) = *self.split[cap].get(l)?;
        if !value.is_finite() {
            return None;
        }
        let w = self.idx.len();
        let mut launches = vec![0u32; n_windows];
        let mut l = l;
        for c in (1..=cap).rev() {
            let m = self.split[c][l].1;
            if m > 0 {
                let (prefix, from) = &self.chains[c - 1];
                let mut k = prefix[m - 1][w - 1].1;
                launches[self.idx[k]] += 1;
                for level in (1..m).rev() {
                    k = from[level][k].expect("a chain of more than one launch steps back");
                    launches[self.idx[k]] += 1;
                }
            }
            l -= m;
        }
        Some((value, launches))
    }
}

/// [`plan_with_lots`], counted, along one direction.
fn counted_lots_along(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    lots: LotChains,
) -> (CampaignPlan, bool) {
    let mut furthest = plan_of(nominal_b, windows, vec![0; windows.len()]);
    // Fewer launches always fit where more do (dropping one only moves the later
    // ones' releases earlier), so the first misfit ends it.
    for l in 1usize.. {
        let Some((_, launches)) = lots.best(l, windows.len()) else {
            break;
        };
        let p = plan_of(nominal_b, windows, launches);
        if p.predicted_impact_parameter() >= target_b {
            return (p, true);
        }
        if p.predicted_impact_parameter() > furthest.predicted_impact_parameter() {
            furthest = p;
        }
    }
    (furthest, false)
}

/// [`plan_with_lots`], outside the cap, along one direction: `outside_cap_along`
/// with the built launches under the lots.
fn outside_cap_lots_along(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    built: LotChains,
    u: Vector2<f64>,
    stock: &Stock,
) -> (CampaignPlan, bool) {
    let best = (0..windows.len())
        .map(|i| (i, windows[i].shift_per_launch.dot(&u)))
        .filter(|&(_, x)| x > 0.0)
        .max_by(|a, b| a.1.total_cmp(&b.1));
    let stock_max = if best.is_some() {
        (stock.size as usize).min(10_000)
    } else {
        0
    };
    let mut furthest = plan_of(nominal_b, windows, vec![0; windows.len()]);
    for l in 1usize.. {
        // The split of `l` that pushes furthest along `u`; none at all means no
        // larger `l` fits either (built launches that fit still fit one fewer).
        let mut top: Option<(f64, usize, Vec<u32>)> = None;
        for k in 0..=l.min(stock_max) {
            let Some((bv, launches)) = built.best(l - k, windows.len()) else {
                continue;
            };
            let val = bv + k as f64 * best.map_or(0.0, |x| x.1);
            if top.as_ref().is_none_or(|(tv, _, _)| val > *tv) {
                top = Some((val, k, launches));
            }
        }
        let Some((_, k, mut launches)) = top else {
            break;
        };
        if let Some((i, _)) = best {
            launches[i] += k as u32;
        }
        let p = plan_of(nominal_b, windows, launches);
        if p.predicted_impact_parameter() >= target_b {
            return (p, true);
        }
        if p.predicted_impact_parameter() > furthest.predicted_impact_parameter() {
            furthest = p;
        }
    }
    (furthest, false)
}

/// Greedy fill along one direction: largest positive shift along `u` first, one
/// launch at a time while the window's period has room, stopping the moment `|B|`
/// clears the target. Returns the plan and whether it reached.
fn fill_along(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    cap: u32,
    u: Vector2<f64>,
) -> (CampaignPlan, bool) {
    let mut order: Vec<(usize, f64)> = windows
        .iter()
        .enumerate()
        .map(|(i, w)| (i, w.shift_per_launch.dot(&u)))
        .filter(|&(_, along)| along > 0.0)
        .collect();
    order.sort_by(|a, b| b.1.total_cmp(&a.1));

    let mut launches = vec![0u32; windows.len()];
    let mut used: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
    let mut b = nominal_b;
    let mut total = 0u32;
    for (i, _) in order {
        let in_period = used.entry(windows[i].period).or_insert(0);
        while *in_period < cap {
            *in_period += 1;
            b += windows[i].shift_per_launch;
            launches[i] += 1;
            total += 1;
            if b.norm() >= target_b {
                return (
                    CampaignPlan {
                        launches,
                        total_launches: total,
                        predicted_b: b,
                    },
                    true,
                );
            }
        }
    }
    (
        CampaignPlan {
            launches,
            total_launches: total,
            predicted_b: b,
        },
        false,
    )
}

// --- Through a parking orbit ---------------------------------------------------

/// One launch flown through a parking orbit: up on `launch_epoch`, out on window
/// `departs_with`'s launch date, through that window's transfer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParkedLaunch {
    /// When the rocket lifts the stack to the parking orbit — the date the launch
    /// cap counts.
    pub launch_epoch: Epoch,
    /// Index into the `windows` the choices were made from: the transfer the stack
    /// leaves on, and the date it leaves.
    pub departs_with: usize,
    /// What is left of the stack after the wait in orbit, as a fraction:
    /// `e^(−k · wait)` for the caller's decay rate `k` (drag make-up,
    /// [`crate::station_keeping`]); `1` with no decay. The push is the departure
    /// window's scale times this.
    pub wait_factor: f64,
}

/// Every parked launch a plan under a rolling cap could need, given the windows a
/// launch could leave through — so that [`plan_campaign_rolling`], handed the
/// direct windows **and** these (as [`parked_window`]s), plans exactly over both
/// ways of flying a launch.
///
/// `scale[i]` is what a parked launch leaving through window `i` pushes, as a
/// fraction of what a direct launch through it pushes (the two masses' ratio — the
/// push is linear in mass); `0` means no parked launch leaves through it. Parked
/// launches go up no earlier than `earliest` and no later than `latest`.
///
/// # Why these dates are enough
/// A parked launch's worth along any push direction can only fall as its launch
/// date moves later — later, fewer departures are still ahead of it. So in a best
/// arrangement every parked launch sits as early as its chain allows (see
/// [`plan_campaign_rolling`]): either at `earliest`, or exactly one period after
/// the launch before it in its chain. Chasing that back, every parked date is
/// `earliest` or a direct window's date plus a whole number of periods — and those
/// are the dates generated. Under a limited stock ([`plan_campaign_stocked`]) a
/// chain can also start at the build date, the first a launch outside the stock can
/// go, so the caller passes it in `also_from`: it seeds dates at itself and whole
/// periods on, as `earliest` does. Each sits a whole number of periods after its base only
/// to rounding, which [`ROLLING_SLACK_S`] absorbs on both sides of it.
///
/// # Why these departures are enough
/// At one date the planner wants, along its push direction `û`, the departure with
/// the largest shift along `û`. Over every `û` those are exactly the vertices of the
/// convex hull of the candidate shifts (with the origin), so only hull vertices are
/// kept — usually a few per date, where every later window would be dozens.
///
/// # Waiting costs: `decay_per_s`
/// A stack in a low orbit pays drag for every day it waits, so what is left of it
/// is `e^(−k · wait)` - `k = decay_per_s`, from [`crate::station_keeping`]. With
/// `k > 0` the dates above are the wrong ones: for a fixed departure, a parked
/// launch's push now *grows* as its date moves later (a shorter wait), so in a best
/// arrangement each sits as **late** as its chain allows - on its departure's own
/// date, or exactly one period before the next launch in its chain, or at
/// `latest`. Chasing that forward, every date is a departure's date or `latest`
/// minus whole periods (from zero), or a direct window's date minus whole periods
/// (from one) - and with `k > 0` those are the only dates generated (the chains
/// only ever use launches that push along their direction, and for those a later
/// date is strictly better; brute force pins it). Lower bounds - `earliest`, the
/// build date, a production line's lots - only ever bind a launch that wants to be
/// earlier, so `also_from` is not needed then. At each date the hull is taken of
/// the waits' scaled shifts. `k = 0` is the free wait, exactly as before.
#[allow(clippy::too_many_arguments)]
pub fn parked_launches(
    windows: &[CampaignWindow],
    scale: &[f64],
    earliest: Epoch,
    also_from: &[Epoch],
    latest: Epoch,
    period_s: f64,
    decay_per_s: f64,
) -> Result<Vec<ParkedLaunch>, CampaignError> {
    if !(decay_per_s.is_finite() && decay_per_s >= 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "the wait's decay rate must be finite and >= 0 per s (got {decay_per_s})"
        )));
    }
    if scale.len() != windows.len() {
        return Err(CampaignError::InvalidInput(format!(
            "{} scales for {} windows",
            scale.len(),
            windows.len()
        )));
    }
    if !(period_s.is_finite() && period_s > 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "the period must be finite and > 0 s (got {period_s})"
        )));
    }
    if let Some(i) = scale.iter().position(|s| !(s.is_finite() && *s >= 0.0)) {
        return Err(CampaignError::InvalidInput(format!(
            "window {i}'s parked scale must be finite and >= 0 (got {})",
            scale[i]
        )));
    }
    let (t_lo, t_hi) = (
        earliest.tdb_seconds_past_j2000(),
        latest.tdb_seconds_past_j2000(),
    );
    let when = |i: usize| windows[i].launch_epoch.tdb_seconds_past_j2000();
    let departures: Vec<usize> = (0..windows.len())
        .filter(|&i| scale[i] > 0.0 && windows[i].shift_per_launch.norm() > 0.0)
        .collect();
    let Some(last_out) = departures.iter().map(|&i| when(i)).reduce(f64::max) else {
        return Ok(Vec::new());
    };
    let all_dates: Vec<f64> = (0..windows.len()).map(when).collect();
    let seeds: Vec<f64> = also_from
        .iter()
        .map(|e| e.tdb_seconds_past_j2000())
        .collect();
    let stop = t_hi.min(last_out);
    let dates = if decay_per_s > 0.0 {
        let mut anchors: Vec<f64> = departures.iter().map(|&i| when(i)).collect();
        anchors.push(stop);
        parked_launch_dates_late(&anchors, &all_dates, t_lo, stop, period_s)
    } else {
        parked_launch_dates(&all_dates, t_lo, &seeds, stop, period_s)
    };

    let mut out = Vec::new();
    for t in dates {
        let ahead: Vec<usize> = departures
            .iter()
            .copied()
            .filter(|&i| when(i) >= t)
            .collect();
        let factor = |i: usize| (-decay_per_s * (when(i) - t)).exp();
        let shifts: Vec<Vector2<f64>> = ahead
            .iter()
            .map(|&i| scale[i] * factor(i) * windows[i].shift_per_launch)
            .collect();
        for k in hull_vertices(&shifts) {
            out.push(ParkedLaunch {
                launch_epoch: Epoch::from_tdb_seconds_past_j2000(t),
                departs_with: ahead[k],
                wait_factor: factor(ahead[k]),
            });
        }
    }
    Ok(out)
}

/// The dates a parked launch can need when waiting costs mass (see
/// [`parked_launches`], `decay_per_s > 0`): each of `anchors` (departure dates and
/// the last launch date) minus whole periods from zero, and each direct window's
/// date in `window_dates` minus whole periods from one - the window dates are
/// direct launches, so a parked launch sits one period before them. Only dates from
/// `earliest` to `stop`, sorted, each once. All TDB s.
///
/// A period that is not finite and > 0 gives no dates.
pub fn parked_launch_dates_late(
    anchors: &[f64],
    window_dates: &[f64],
    earliest: f64,
    stop: f64,
    period_s: f64,
) -> Vec<f64> {
    if !(period_s.is_finite() && period_s > 0.0) {
        return Vec::new();
    }
    let mut dates: Vec<f64> = Vec::new();
    let mut back = |base: f64, first: u32| {
        let mut k = first;
        loop {
            let t = base - f64::from(k) * period_s;
            if t < earliest {
                break;
            }
            if t <= stop {
                dates.push(t);
            }
            k += 1;
        }
    };
    for &t in anchors {
        back(t, 0);
    }
    for &t in window_dates {
        back(t, 1);
    }
    dates.sort_by(f64::total_cmp);
    dates.dedup();
    dates
}

/// Every date a parked launch can need (see [`parked_launches`] for why): `earliest`
/// and each date in `also_from` plus whole periods (from zero), and each direct
/// window's date in `window_dates` plus whole periods - the window dates themselves
/// are direct launches, so a parked launch starts one period after them. Only dates
/// from `earliest` to `stop`, sorted, each once. All TDB s.
///
/// A period that is not finite and > 0 gives no dates.
pub fn parked_launch_dates(
    window_dates: &[f64],
    earliest: f64,
    also_from: &[f64],
    stop: f64,
    period_s: f64,
) -> Vec<f64> {
    if !(period_s.is_finite() && period_s > 0.0) {
        return Vec::new();
    }
    let mut dates: Vec<f64> = Vec::new();
    let mut from = |base: f64, first: u32| {
        let mut k = first;
        loop {
            let t = base + f64::from(k) * period_s;
            if t > stop {
                break;
            }
            if t >= earliest {
                dates.push(t);
            }
            k += 1;
        }
    };
    from(earliest, 0);
    for &t in also_from {
        from(t, 0);
    }
    for &t in window_dates {
        from(t, 1);
    }
    dates.sort_by(f64::total_cmp);
    dates.dedup();
    dates
}

/// The planner's view of one parked launch: launched on its own date (which the cap
/// counts), arriving with its departure window, pushing `scale ×` that window's
/// per-launch push. `period` is the caller's label, as for any window.
pub fn parked_window(
    departure: &CampaignWindow,
    scale: f64,
    launch_epoch: Epoch,
    period: u32,
) -> CampaignWindow {
    CampaignWindow {
        launch_epoch,
        period,
        arrival_epoch: departure.arrival_epoch,
        impulse_per_launch: scale * departure.impulse_per_launch,
        shift_per_launch: scale * departure.shift_per_launch,
    }
}

/// Which `pool` windows a parked launch can leave through, given the `flown` ones
/// and the dates it can go up on (`launch_dates`, from [`parked_launch_dates`]):
/// each window is `(date, key)` - its date (TDB s) and its worth to a parked
/// launch, the larger the better. Returns the indices into `pool`, ascending.
///
/// # Why this is the set
/// The cap counts a parked launch's **launch** date, not its departure, and any
/// number of parked stacks may leave through one window - so a parked launch that
/// went up on date `t` takes the best window dated `t` or later, and nothing else.
/// The pool windows returned are the ones that are that best for some launch date.
/// A window only as good as (or worse than) another at or after its own date is
/// never one; nor is a window that is the best from some date on, but only from
/// dates no launch can go up on - which is why the launch dates are an argument.
/// Without them the answer is every window that beats everything after it, and
/// since a window's worth grows with its lead, read from the last date backwards
/// that is most of the pool.
///
/// A key of 0 or less never counts. A tie goes to a flown window (at any date a
/// launch reaches both), then to the later pool window, then to the lower index.
pub fn best_ahead(launch_dates: &[f64], flown: &[(f64, f64)], pool: &[(f64, f64)]) -> Vec<usize> {
    // The staircase, read from the last date backwards: every window that beats
    // everything at or after its date, as (date, None = flown | Some(pool index)).
    // Sorted latest first, then the highest key, flown before pool, lower index
    // first - so the first entry of a date is the only one that can beat the best.
    let mut all: Vec<(f64, f64, Option<usize>)> = flown
        .iter()
        .map(|&(t, k)| (t, k, None))
        .chain(pool.iter().enumerate().map(|(i, &(t, k))| (t, k, Some(i))))
        .collect();
    all.sort_by(|a, b| {
        b.0.total_cmp(&a.0)
            .then(b.1.total_cmp(&a.1))
            .then(a.2.cmp(&b.2))
    });
    // A flown window that only *ties* the best after it still takes the step: a
    // launch that can reach both needs nothing flown.
    let mut best = 0.0_f64;
    let mut stairs: Vec<(f64, Option<usize>)> = Vec::new();
    for (t, k, i) in all {
        if k > best || (i.is_none() && k == best && k > 0.0) {
            best = k;
            stairs.push((t, i));
        }
    }
    // Earliest first: the best window at or after `t` is the first step dated `t`
    // or later.
    stairs.reverse();
    let mut out: Vec<usize> = launch_dates
        .iter()
        .filter_map(|&t| {
            let j = stairs.partition_point(|s| s.0 < t);
            stairs.get(j).and_then(|s| s.1)
        })
        .collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// Indices of the points that are vertices of the convex hull of `points` and the
/// origin — the points that are the furthest along *some* direction. Points on a
/// hull edge, inside it, at the origin, or repeated are dropped (a repeat keeps its
/// first index).
fn hull_vertices(points: &[Vector2<f64>]) -> Vec<usize> {
    // Origin first, then the points, sorted by (x, y) for Andrew's monotone chain.
    let mut all: Vec<(Vector2<f64>, Option<usize>)> = vec![(Vector2::zeros(), None)];
    all.extend(points.iter().enumerate().map(|(i, &p)| (p, Some(i))));
    all.sort_by(|a, b| a.0.x.total_cmp(&b.0.x).then(a.0.y.total_cmp(&b.0.y)));
    all.dedup_by(|b, a| a.0 == b.0);
    if all.len() < 2 {
        return Vec::new();
    }
    let cross = |o: Vector2<f64>, a: Vector2<f64>, b: Vector2<f64>| {
        (a - o).x * (b - o).y - (a - o).y * (b - o).x
    };
    // Lower hull left to right, then upper hull back; `<= 0` drops points on an edge.
    let mut hull: Vec<(Vector2<f64>, Option<usize>)> = Vec::with_capacity(2 * all.len());
    for &p in &all {
        while hull.len() >= 2 && cross(hull[hull.len() - 2].0, hull[hull.len() - 1].0, p.0) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    let lower = hull.len() + 1;
    for &p in all.iter().rev().skip(1) {
        while hull.len() >= lower
            && cross(hull[hull.len() - 2].0, hull[hull.len() - 1].0, p.0) <= 0.0
        {
            hull.pop();
        }
        hull.push(p);
    }
    hull.pop();
    let mut out: Vec<usize> = hull.into_iter().filter_map(|(_, i)| i).collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// The impulses a plan applies, in the order
/// [`DeflectionScenario::evaluate_campaign`](crate::deflection::DeflectionScenario::evaluate_campaign)
/// wants them: sorted by arrival epoch, each window's launches folded into one
/// impulse of `n ×` its per-launch push, and windows with no launch left out.
///
/// Folding is exact, not an approximation: launches that arrive together push
/// together.
pub fn campaign_impulses(
    windows: &[CampaignWindow],
    launches: &[u32],
) -> Vec<(Epoch, Vector3<f64>)> {
    let mut out: Vec<(Epoch, Vector3<f64>)> = windows
        .iter()
        .zip(launches)
        .filter(|&(_, &n)| n > 0)
        .map(|(w, &n)| (w.arrival_epoch, f64::from(n) * w.impulse_per_launch))
        .collect();
    out.sort_by(|a, b| {
        a.0.tdb_seconds_past_j2000()
            .total_cmp(&b.0.tdb_seconds_past_j2000())
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A window alone in its own period (numbered by its day, so two windows in
    /// one test never share a period by accident).
    fn window(t_days: f64, shift: (f64, f64)) -> CampaignWindow {
        in_period(t_days, t_days as u32, shift)
    }

    fn in_period(t_days: f64, period: u32, shift: (f64, f64)) -> CampaignWindow {
        let e = Epoch::from_tdb_seconds_past_j2000(t_days * 86_400.0);
        CampaignWindow {
            launch_epoch: e,
            period,
            arrival_epoch: e,
            impulse_per_launch: Vector3::new(shift.0, shift.1, 0.0) * 1.0e-9,
            shift_per_launch: Vector2::new(shift.0, shift.1),
        }
    }

    fn planned(o: CampaignOutcome) -> CampaignPlan {
        match o {
            CampaignOutcome::Planned(p) => p,
            other => panic!("expected a plan, got {other:?}"),
        }
    }

    #[test]
    fn a_nominal_already_clear_needs_no_launch() {
        let w = [window(0.0, (0.0, 100.0))];
        let o = plan_campaign(Vector2::new(0.0, 6_000.0), 5_000.0, &w, 3).unwrap();
        assert_eq!(o, CampaignOutcome::AlreadyClear);
    }

    /// Strongest window first, and the count is exact: from `ζ = 1000` the 1000-m
    /// window reaches 5000 in four launches under a cap of five, and under a cap
    /// of two the weaker window cannot make up the rest.
    #[test]
    fn takes_the_strongest_window_first_and_respects_the_cap() {
        let w = [window(0.0, (0.0, 300.0)), window(10.0, (0.0, 1_000.0))];
        let p = planned(plan_campaign(Vector2::new(0.0, 1_000.0), 5_000.0, &w, 5).unwrap());
        assert_eq!(p.launches, vec![0, 4]);
        assert_eq!(p.total_launches, 4);
        assert!((p.predicted_impact_parameter() - 5_000.0).abs() < 1e-9);

        match plan_campaign(Vector2::new(0.0, 1_000.0), 5_000.0, &w, 2).unwrap() {
            CampaignOutcome::Unreachable(best) => {
                assert_eq!(best.launches, vec![2, 2]);
                assert!((best.predicted_impact_parameter() - 3_600.0).abs() < 1e-9);
            }
            other => panic!("expected unreachable, got {other:?}"),
        }
    }

    /// Opposite pushes are never mixed: from a dead-centre hit, the planner goes
    /// one way with the windows that push that way, and the other window stays at
    /// zero — using it would cancel launches already paid for.
    #[test]
    fn opposite_pushes_are_not_mixed() {
        let w = [window(0.0, (0.0, 500.0)), window(10.0, (0.0, -800.0))];
        let p = planned(plan_campaign(Vector2::zeros(), 2_000.0, &w, 3).unwrap());
        assert_eq!(p.launches, vec![0, 3]);
        assert!((p.predicted_b.y + 2_400.0).abs() < 1e-9);
    }

    /// On collinear shifts the greedy fill is the true minimum. Brute force over
    /// every allocation of up to `cap` launches to three windows, both signs, many
    /// targets — the planner's count must equal the smallest count that reaches.
    #[test]
    fn greedy_matches_brute_force_on_collinear_shifts() {
        let shifts = [370.0, -910.0, 640.0, -150.0];
        let w: Vec<CampaignWindow> = shifts
            .iter()
            .enumerate()
            .map(|(i, &s)| window(i as f64 * 30.0, (0.0, s)))
            .collect();
        let cap = 3u32;
        for b0 in [0.0, 400.0, -700.0] {
            for target in [500.0, 1_300.0, 2_200.0, 2_900.0, 4_000.0] {
                let mut brute: Option<u32> = None;
                for a in 0..=cap {
                    for b in 0..=cap {
                        for c in 0..=cap {
                            for d in 0..=cap {
                                let z = b0
                                    + a as f64 * shifts[0]
                                    + b as f64 * shifts[1]
                                    + c as f64 * shifts[2]
                                    + d as f64 * shifts[3];
                                if z.abs() >= target {
                                    let n = a + b + c + d;
                                    brute = Some(brute.map_or(n, |m| m.min(n)));
                                }
                            }
                        }
                    }
                }
                let got = plan_campaign(Vector2::new(0.0, b0), target, &w, cap).unwrap();
                match (brute, got) {
                    (Some(0), CampaignOutcome::AlreadyClear) => {}
                    (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                        p.total_launches, n,
                        "b0={b0} target={target}: planner {} vs brute force {n}",
                        p.total_launches
                    ),
                    (None, CampaignOutcome::Unreachable(_)) => {}
                    (brute, got) => panic!("b0={b0} target={target}: brute {brute:?} vs {got:?}"),
                }
            }
        }
    }

    /// The cap is shared across a period, not granted per window: two windows in
    /// one period get `cap` launches between them, and the stronger one takes them.
    /// Under a per-window cap the same input would plan 2 + 2.
    #[test]
    fn windows_in_one_period_share_its_cap() {
        let w = [
            in_period(0.0, 7, (0.0, 600.0)),
            in_period(20.0, 7, (0.0, 900.0)),
            in_period(400.0, 8, (0.0, 500.0)),
        ];
        let p = planned(plan_campaign(Vector2::zeros(), 2_700.0, &w, 2).unwrap());
        assert_eq!(p.launches, vec![0, 2, 2]);
        assert_eq!(p.total_launches, 4);
        match plan_campaign(Vector2::zeros(), 3_500.0, &w, 2).unwrap() {
            CampaignOutcome::Unreachable(best) => {
                assert_eq!(best.launches, vec![0, 2, 2]);
                assert!((best.predicted_b.y - 2_800.0).abs() < 1e-9);
            }
            other => panic!("expected unreachable, got {other:?}"),
        }
    }

    /// Greedy stays the true minimum when windows share periods (a partition
    /// constraint). Five collinear windows in three periods — one period holding two
    /// same-sign windows, one holding a pair of opposite signs — brute-forced over
    /// every allocation that keeps each period within its cap.
    #[test]
    fn greedy_matches_brute_force_with_shared_periods() {
        let spec: [(u32, f64); 5] = [(0, 370.0), (0, 640.0), (1, -910.0), (1, 450.0), (2, -150.0)];
        let w: Vec<CampaignWindow> = spec
            .iter()
            .enumerate()
            .map(|(i, &(per, s))| in_period(i as f64 * 30.0, per, (0.0, s)))
            .collect();
        for cap in [1u32, 2, 3] {
            for b0 in [0.0, 400.0, -700.0] {
                for target in [500.0, 1_300.0, 2_200.0, 2_900.0, 4_000.0] {
                    let mut brute: Option<u32> = None;
                    let r = cap + 1;
                    for code in 0..r.pow(5) {
                        let n: Vec<u32> = (0..5).map(|k| (code / r.pow(k)) % r).collect();
                        let per = |p: u32| -> u32 {
                            (0..5).filter(|&k| spec[k].0 == p).map(|k| n[k]).sum()
                        };
                        if (0..3).any(|p| per(p) > cap) {
                            continue;
                        }
                        let z = b0 + (0..5).map(|k| f64::from(n[k]) * spec[k].1).sum::<f64>();
                        if z.abs() >= target {
                            let t: u32 = n.iter().sum();
                            brute = Some(brute.map_or(t, |m| m.min(t)));
                        }
                    }
                    let got = plan_campaign(Vector2::new(0.0, b0), target, &w, cap).unwrap();
                    match (brute, got) {
                        (Some(0), CampaignOutcome::AlreadyClear) => {}
                        (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                            p.total_launches, n,
                            "cap={cap} b0={b0} target={target}: planner {} vs brute {n}",
                            p.total_launches
                        ),
                        (None, CampaignOutcome::Unreachable(_)) => {}
                        (brute, got) => {
                            panic!("cap={cap} b0={b0} target={target}: brute {brute:?} vs {got:?}")
                        }
                    }
                }
            }
        }
    }

    // --- The rolling cap -------------------------------------------------------

    const DAY: f64 = 86_400.0;

    /// Launches of a plan as `(epoch s, n)`, for [`busiest_rolling_count`].
    fn launch_list(w: &[CampaignWindow], p: &CampaignPlan) -> Vec<(f64, u32)> {
        w.iter()
            .zip(&p.launches)
            .filter(|(_, &n)| n > 0)
            .map(|(w, &n)| (w.launch_epoch.tdb_seconds_past_j2000(), n))
            .collect()
    }

    /// The case a greedy fill gets wrong: one strong window in the middle blocks
    /// both neighbours, which together are worth more. With one launch per 100 days,
    /// greedy takes the 1 000 and is done; the two 600s, 100 days apart, reach 1 100.
    #[test]
    fn the_rolling_planner_skips_a_strong_window_that_blocks_two() {
        let w = [
            window(0.0, (0.0, 600.0)),
            window(50.0, (0.0, 1_000.0)),
            window(100.0, (0.0, 600.0)),
        ];
        let p =
            planned(plan_campaign_rolling(Vector2::zeros(), 1_100.0, &w, 1, 100.0 * DAY).unwrap());
        assert_eq!(p.launches, vec![1, 0, 1]);
        // Exactly one window length apart is allowed: the window is half-open.
        assert_eq!(busiest_rolling_count(&launch_list(&w, &p), 100.0 * DAY), 1);
        // And one second short of it is not.
        let tight = [
            window(0.0, (0.0, 600.0)),
            window(100.0 - 1.0 / DAY, (0.0, 600.0)),
        ];
        match plan_campaign_rolling(Vector2::zeros(), 1_100.0, &tight, 1, 100.0 * DAY).unwrap() {
            CampaignOutcome::Unreachable(best) => assert_eq!(best.total_launches, 1),
            other => panic!("expected unreachable, got {other:?}"),
        }
    }

    /// The cap stacks on one date: N launches the same day are N in that window.
    #[test]
    fn the_rolling_cap_allows_n_on_one_date() {
        let w = [window(0.0, (0.0, 500.0))];
        let p =
            planned(plan_campaign_rolling(Vector2::zeros(), 1_400.0, &w, 3, 365.25 * DAY).unwrap());
        assert_eq!(p.launches, vec![3]);
        match plan_campaign_rolling(Vector2::zeros(), 1_600.0, &w, 3, 365.25 * DAY).unwrap() {
            CampaignOutcome::Unreachable(best) => assert_eq!(best.launches, vec![3]),
            other => panic!("expected unreachable, got {other:?}"),
        }
    }

    /// Exact against brute force: six collinear windows of both signs at uneven
    /// dates, every allocation of up to `cap` launches per window that keeps every
    /// rolling 100-day window within `cap`, many targets. The planner's count must
    /// be the smallest that reaches, and every plan it returns must obey the cap.
    #[test]
    fn the_rolling_planner_matches_brute_force() {
        let spec: [(f64, f64); 6] = [
            (0.0, 640.0),
            (30.0, -910.0),
            (70.0, 370.0),
            (95.0, 820.0),
            (160.0, -150.0),
            (240.0, 450.0),
        ];
        let w: Vec<CampaignWindow> = spec.iter().map(|&(t, s)| window(t, (0.0, s))).collect();
        let span = 100.0 * DAY;
        for cap in [1u32, 2, 3] {
            for b0 in [0.0, 400.0, -700.0] {
                for target in [500.0, 1_300.0, 2_200.0, 2_900.0, 4_000.0, 6_000.0] {
                    let r = cap + 1;
                    let mut brute: Option<u32> = None;
                    for code in 0..r.pow(6) {
                        let n: Vec<u32> = (0..6).map(|k| (code / r.pow(k)) % r).collect();
                        let list: Vec<(f64, u32)> = (0..6)
                            .filter(|&k| n[k] > 0)
                            .map(|k| (spec[k].0 * DAY, n[k]))
                            .collect();
                        if busiest_rolling_count(&list, span) > cap {
                            continue;
                        }
                        let z = b0 + (0..6).map(|k| f64::from(n[k]) * spec[k].1).sum::<f64>();
                        if z.abs() >= target {
                            let t: u32 = n.iter().sum();
                            brute = Some(brute.map_or(t, |m| m.min(t)));
                        }
                    }
                    let got = plan_campaign_rolling(Vector2::new(0.0, b0), target, &w, cap, span)
                        .unwrap();
                    if let CampaignOutcome::Planned(p) | CampaignOutcome::Unreachable(p) = &got {
                        assert!(
                            busiest_rolling_count(&launch_list(&w, p), span) <= cap,
                            "cap={cap} b0={b0} target={target}: plan breaks the cap: {p:?}"
                        );
                    }
                    match (brute, got) {
                        (Some(0), CampaignOutcome::AlreadyClear) => {}
                        (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                            p.total_launches, n,
                            "cap={cap} b0={b0} target={target}: planner {} vs brute {n}",
                            p.total_launches
                        ),
                        (None, CampaignOutcome::Unreachable(_)) => {}
                        (brute, got) => {
                            panic!("cap={cap} b0={b0} target={target}: brute {brute:?} vs {got:?}")
                        }
                    }
                }
            }
        }
    }

    // --- A limited stock --------------------------------------------------------

    /// Seven windows, three before the build date: more good early windows than a
    /// small stock covers, so the stock binds.
    const STOCK_SPEC: [(f64, f64); 7] = [
        (0.0, 640.0),
        (30.0, 910.0),
        (70.0, 370.0),
        (130.0, 820.0),
        (160.0, -150.0),
        (240.0, 450.0),
        (300.0, 300.0),
    ];
    const STOCK_BUILD_DAYS: f64 = 140.0;

    fn stock_windows() -> Vec<CampaignWindow> {
        STOCK_SPEC
            .iter()
            .map(|&(t, s)| window(t, (0.0, s)))
            .collect()
    }

    fn stock(size: u32, mode: StockMode) -> Stock {
        Stock {
            size,
            build_from: Epoch::from_tdb_seconds_past_j2000(STOCK_BUILD_DAYS * DAY),
            mode,
            built_per_period: UNLIMITED_BUILDS,
        }
    }

    /// The fewest launches by exhaustion, or `None`. `counted`: one rolling cap over
    /// everything and at most `size` launches before the build date. Otherwise the
    /// built launches (none before the build date) take the cap and up to `size`
    /// stocked ones go through any one window, uncapped.
    fn stock_brute(cap: u32, size: u32, counted: bool, b0: f64, target: f64) -> Option<u32> {
        let n_w = STOCK_SPEC.len();
        let span = 100.0 * DAY;
        let r = cap + 1;
        let early = |k: usize| STOCK_SPEC[k].0 < STOCK_BUILD_DAYS;
        let mut best: Option<u32> = None;
        for code in 0..r.pow(n_w as u32) {
            let n: Vec<u32> = (0..n_w).map(|k| (code / r.pow(k as u32)) % r).collect();
            let list: Vec<(f64, u32)> = (0..n_w)
                .filter(|&k| n[k] > 0)
                .map(|k| (STOCK_SPEC[k].0 * DAY, n[k]))
                .collect();
            if busiest_rolling_count(&list, span) > cap {
                continue;
            }
            let early_n: u32 = (0..n_w).filter(|&k| early(k)).map(|k| n[k]).sum();
            let z_built = b0
                + (0..n_w)
                    .map(|k| f64::from(n[k]) * STOCK_SPEC[k].1)
                    .sum::<f64>();
            let total_built: u32 = n.iter().sum();
            if counted {
                if early_n <= size && z_built.abs() >= target {
                    best = Some(best.map_or(total_built, |m| m.min(total_built)));
                }
                continue;
            }
            if early_n > 0 {
                continue; // built launches cannot go before the build date
            }
            for k_stock in 0..=size {
                for &(_, sz) in STOCK_SPEC.iter() {
                    let z = z_built + f64::from(k_stock) * sz;
                    if z.abs() >= target {
                        let t = total_built + k_stock;
                        best = Some(best.map_or(t, |m| m.min(t)));
                    }
                }
            }
        }
        best
    }

    #[test]
    fn the_stocked_planner_matches_brute_force() {
        let w = stock_windows();
        let span = 100.0 * DAY;
        let build = STOCK_BUILD_DAYS * DAY;
        let mut bound = 0;
        for counted in [true, false] {
            let mode = if counted {
                StockMode::Counted
            } else {
                StockMode::OutsideCap
            };
            for cap in [1u32, 2] {
                for size in [0u32, 1, 2, 3] {
                    for b0 in [0.0, 400.0] {
                        for target in [900.0, 1_700.0, 2_600.0, 3_500.0, 5_000.0] {
                            let brute = stock_brute(cap, size, counted, b0, target);
                            let got = plan_campaign_stocked(
                                Vector2::new(0.0, b0),
                                target,
                                &w,
                                cap,
                                span,
                                &stock(size, mode),
                            )
                            .unwrap();
                            let tag =
                                format!("{mode:?} cap={cap} size={size} b0={b0} target={target}");
                            if let CampaignOutcome::Planned(p) | CampaignOutcome::Unreachable(p) =
                                &got
                            {
                                let early: u32 = w
                                    .iter()
                                    .zip(&p.launches)
                                    .filter(|(x, _)| {
                                        x.launch_epoch.tdb_seconds_past_j2000() < build
                                    })
                                    .map(|(_, &n)| n)
                                    .sum();
                                assert!(
                                    early <= size,
                                    "{tag}: {early} launches before the build date"
                                );
                                if counted {
                                    assert!(
                                        busiest_rolling_count(&launch_list(&w, p), span) <= cap,
                                        "{tag}: breaks the cap"
                                    );
                                }
                            }
                            // The stock binds somewhere: a bigger one does better.
                            if counted && size < 3 {
                                let more = stock_brute(cap, 3, true, b0, target);
                                if more.is_some() && (brute.is_none() || more < brute) {
                                    bound += 1;
                                }
                            }
                            match (brute, got) {
                                (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                                    p.total_launches, n,
                                    "{tag}: planner {} vs brute {n}",
                                    p.total_launches
                                ),
                                (None, CampaignOutcome::Unreachable(_)) => {}
                                (brute, got) => panic!("{tag}: brute {brute:?} vs {got:?}"),
                            }
                        }
                    }
                }
            }
        }
        assert!(
            bound > 0,
            "no case where the stock binds - the brute force never tested the constraint"
        );
    }

    #[test]
    fn a_stock_no_plan_can_exhaust_is_the_rolling_planner() {
        let w = stock_windows();
        let span = 100.0 * DAY;
        for cap in [1u32, 2, 3] {
            for target in [900.0, 2_600.0, 5_000.0] {
                let rolling =
                    plan_campaign_rolling(Vector2::zeros(), target, &w, cap, span).unwrap();
                let all = plan_campaign_stocked(
                    Vector2::zeros(),
                    target,
                    &w,
                    cap,
                    span,
                    &stock(u32::MAX, StockMode::Counted),
                )
                .unwrap();
                assert_eq!(all, rolling, "cap={cap} target={target}");
            }
        }
    }

    #[test]
    fn no_stock_is_the_rolling_planner_from_the_build_date() {
        let w = stock_windows();
        let build = STOCK_BUILD_DAYS * DAY;
        let late: Vec<CampaignWindow> = w
            .iter()
            .filter(|x| x.launch_epoch.tdb_seconds_past_j2000() >= build)
            .copied()
            .collect();
        let span = 100.0 * DAY;
        let key = |o: &CampaignOutcome| match o {
            CampaignOutcome::Planned(p) => (true, p.total_launches, p.predicted_impact_parameter()),
            CampaignOutcome::Unreachable(p) => {
                (false, p.total_launches, p.predicted_impact_parameter())
            }
            CampaignOutcome::AlreadyClear => (true, 0, 0.0),
        };
        for mode in [StockMode::Counted, StockMode::OutsideCap] {
            for cap in [1u32, 2, 3] {
                for target in [500.0, 1_200.0, 2_000.0, 4_000.0] {
                    let from_build =
                        plan_campaign_rolling(Vector2::zeros(), target, &late, cap, span).unwrap();
                    let none = plan_campaign_stocked(
                        Vector2::zeros(),
                        target,
                        &w,
                        cap,
                        span,
                        &stock(0, mode),
                    )
                    .unwrap();
                    let (a, b) = (key(&from_build), key(&none));
                    assert_eq!((a.0, a.1), (b.0, b.1), "{mode:?} cap={cap} target={target}");
                    assert!(
                        (a.2 - b.2).abs() < 1e-6,
                        "{mode:?} cap={cap} target={target}: {a:?} vs {b:?}"
                    );
                }
            }
        }
    }

    // --- A production line -------------------------------------------------------

    /// `STOCK_SPEC` with every rule a lot-limited plan obeys, by exhaustion: the
    /// rolling cap, the stock before the build date (counted) or none of the built
    /// launches there (outside), and at most `stocked + (k+1)·per` launches before
    /// lot k+1 arrives. Outside the cap, up to `size` stocked launches through any one
    /// window, uncapped. The fewest launches that reach, or `None`.
    fn lots_brute(
        cap: u32,
        size: u32,
        per: u32,
        counted: bool,
        b0: f64,
        target: f64,
    ) -> Option<u32> {
        let n_w = STOCK_SPEC.len();
        let span = 100.0 * DAY;
        let build = STOCK_BUILD_DAYS * DAY;
        let r = cap + 1;
        let mut best: Option<u32> = None;
        for code in 0..r.pow(n_w as u32) {
            let n: Vec<u32> = (0..n_w).map(|k| (code / r.pow(k as u32)) % r).collect();
            let list: Vec<(f64, u32)> = (0..n_w)
                .filter(|&k| n[k] > 0)
                .map(|k| (STOCK_SPEC[k].0 * DAY, n[k]))
                .collect();
            if busiest_rolling_count(&list, span) > cap {
                continue;
            }
            let before = |t: f64| -> u32 {
                (0..n_w)
                    .filter(|&k| STOCK_SPEC[k].0 * DAY < t)
                    .map(|k| n[k])
                    .sum()
            };
            let stocked = if counted { size } else { 0 };
            if before(build) > stocked {
                continue;
            }
            let lots_ok = (0..10u32).all(|k| {
                let next = build + f64::from(k + 1) * (span - ROLLING_SLACK_S);
                before(next) <= stocked + per * (k + 1)
            });
            if !lots_ok {
                continue;
            }
            let z = b0
                + (0..n_w)
                    .map(|k| f64::from(n[k]) * STOCK_SPEC[k].1)
                    .sum::<f64>();
            let total: u32 = n.iter().sum();
            let extra = if counted { 0 } else { size };
            for k_stock in 0..=extra {
                for &(_, sz) in STOCK_SPEC.iter() {
                    if (z + f64::from(k_stock) * sz).abs() >= target {
                        let t = total + k_stock;
                        best = Some(best.map_or(t, |m| m.min(t)));
                    }
                }
            }
        }
        best
    }

    fn lots(size: u32, per: u32, mode: StockMode) -> Stock {
        Stock {
            built_per_period: per,
            ..stock(size, mode)
        }
    }

    #[test]
    fn the_production_line_matches_brute_force() {
        let w = stock_windows();
        let span = 100.0 * DAY;
        let mut binds = [0u32; 2];
        for counted in [true, false] {
            let mode = if counted {
                StockMode::Counted
            } else {
                StockMode::OutsideCap
            };
            for cap in [2u32, 3] {
                for per in 1..cap {
                    for size in [0u32, 1, 2] {
                        for b0 in [0.0, 400.0] {
                            for target in [
                                900.0, 1_300.0, 1_700.0, 2_100.0, 2_600.0, 3_000.0, 3_500.0,
                                4_200.0, 5_000.0,
                            ] {
                                let tag = format!(
                                    "{mode:?} cap={cap} per={per} size={size} b0={b0} target={target}"
                                );
                                let brute = lots_brute(cap, size, per, counted, b0, target);
                                let free = lots_brute(cap, size, cap, counted, b0, target);
                                if brute != free {
                                    binds[usize::from(counted)] += 1;
                                }
                                let got = plan_campaign_stocked(
                                    Vector2::new(0.0, b0),
                                    target,
                                    &w,
                                    cap,
                                    span,
                                    &lots(size, per, mode),
                                )
                                .unwrap();
                                match (brute, got) {
                                    (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                                        p.total_launches, n,
                                        "{tag}: planner {} vs brute {n}",
                                        p.total_launches
                                    ),
                                    (None, CampaignOutcome::Unreachable(_)) => {}
                                    (brute, got) => panic!("{tag}: brute {brute:?} vs {got:?}"),
                                }
                            }
                        }
                    }
                }
            }
        }
        // The lots change the answer in both modes, or the brute force never tested
        // them.
        assert!(
            binds.iter().all(|&b| b > 0),
            "cases where the lots bind (outside, counted): {binds:?}"
        );
    }

    /// Random windows with collinear shifts (so `|B|` is a function of the push
    /// along the one direction, and the planners can be compared on counts), some of
    /// them a whole number of periods after the build date - exactly, or a tenth of a
    /// millisecond short, inside the slack the rolling cap reads "a period apart" with.
    fn random_windows(seed: u64, n: usize, build_days: f64) -> Vec<CampaignWindow> {
        let mut x = seed;
        let mut next = move || {
            x = x
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (x >> 33) as f64 / f64::from(1u32 << 31)
        };
        (0..n)
            .map(|_| {
                let t = if next() < 0.4 {
                    let short = if next() < 0.5 { 1.0e-4 / DAY } else { 0.0 };
                    build_days + 100.0 * (next() * 4.0).floor() - short
                } else {
                    next() * 500.0
                };
                let s = if next() < 0.2 { -1.0 } else { 1.0 } * (50.0 + next() * 900.0);
                window(t, (0.0, s))
            })
            .collect()
    }

    fn count(o: &CampaignOutcome) -> Option<u32> {
        match o {
            CampaignOutcome::Planned(p) => Some(p.total_launches),
            CampaignOutcome::AlreadyClear => Some(0),
            CampaignOutcome::Unreachable(_) => None,
        }
    }

    #[test]
    fn lots_at_the_cap_are_the_chains_exactly() {
        // The packing route, forced, against the chains at a production rate that
        // cannot bind: same count, same push, in both modes and at windows sitting
        // exactly a whole number of periods after the build date.
        let span = 100.0 * DAY;
        let build = 140.0;
        for seed in 0..60u64 {
            let w = random_windows(seed, 9, build);
            for cap in [1u32, 2, 3] {
                for size in [0u32, 1, 3] {
                    for mode in [StockMode::Counted, StockMode::OutsideCap] {
                        for target in [700.0, 2_000.0, 4_500.0] {
                            let free = Stock {
                                size,
                                build_from: Epoch::from_tdb_seconds_past_j2000(build * DAY),
                                mode,
                                built_per_period: UNLIMITED_BUILDS,
                            };
                            let chains = plan_campaign_stocked(
                                Vector2::zeros(),
                                target,
                                &w,
                                cap,
                                span,
                                &free,
                            )
                            .unwrap();
                            let packed = plan_with_lots(
                                Vector2::zeros(),
                                target,
                                &w,
                                cap,
                                span,
                                &Stock {
                                    built_per_period: cap,
                                    ..free
                                },
                            );
                            let tag = format!(
                                "seed={seed} cap={cap} size={size} {mode:?} target={target}"
                            );
                            assert_eq!(count(&chains), count(&packed), "{tag}");
                            if let (
                                CampaignOutcome::Planned(a) | CampaignOutcome::Unreachable(a),
                                CampaignOutcome::Planned(b) | CampaignOutcome::Unreachable(b),
                            ) = (&chains, &packed)
                            {
                                let (ia, ib) = (
                                    a.predicted_impact_parameter(),
                                    b.predicted_impact_parameter(),
                                );
                                assert!((ia - ib).abs() < 1e-6, "{tag}: {ia} vs {ib}");
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn a_production_line_sits_between_no_limit_and_no_saving() {
        // No stock: unlimited building can only need fewer launches, and a cap of `B`
        // on every launch (built impactors that cannot wait) can only need more.
        let span = 100.0 * DAY;
        let build = 0.0;
        let mut strictly = 0;
        for seed in 0..200u64 {
            let w = random_windows(seed, 10, build);
            for cap in [2u32, 3, 4] {
                for per in 1..cap {
                    for target in [1_000.0, 2_500.0, 5_000.0] {
                        let s = |per: u32| Stock {
                            size: 0,
                            build_from: Epoch::from_tdb_seconds_past_j2000(build),
                            mode: StockMode::Counted,
                            built_per_period: per,
                        };
                        let plan = |c: u32, per: u32| {
                            plan_campaign_stocked(Vector2::zeros(), target, &w, c, span, &s(per))
                                .unwrap()
                        };
                        let free = count(&plan(cap, UNLIMITED_BUILDS));
                        let lots = count(&plan(cap, per));
                        let no_saving = count(&plan(per, UNLIMITED_BUILDS));
                        let tag = format!("seed={seed} cap={cap} per={per} target={target}");
                        // `None` (short) sorts above every count.
                        let key = |c: Option<u32>| c.unwrap_or(u32::MAX);
                        assert!(key(free) <= key(lots), "{tag}: {free:?} > {lots:?}");
                        assert!(
                            key(lots) <= key(no_saving),
                            "{tag}: {lots:?} > {no_saving:?}"
                        );
                        if key(lots) < key(no_saving) {
                            strictly += 1;
                        }
                    }
                }
            }
        }
        assert!(
            strictly > 0,
            "saving never helped - the comparison tests nothing"
        );
    }

    #[test]
    fn lots_finished_before_the_first_window_are_all_waiting() {
        // A rock found long before the first launch date: one built a period, the
        // first finished 500 days before the only window. Five lots are waiting, so
        // the window can take the whole cap of 4 at once - and with the build date
        // moved up to the window itself, only one.
        let span = 100.0 * DAY;
        let w = [window(1_000.0, (0.0, 10.0))];
        let line = |build_days: f64| Stock {
            size: 0,
            build_from: Epoch::from_tdb_seconds_past_j2000(build_days * DAY),
            mode: StockMode::Counted,
            built_per_period: 1,
        };
        let early = planned(
            plan_campaign_stocked(Vector2::zeros(), 40.0, &w, 4, span, &line(500.0)).unwrap(),
        );
        assert_eq!(early.launches, vec![4]);
        let late =
            plan_campaign_stocked(Vector2::zeros(), 40.0, &w, 4, span, &line(1_000.0)).unwrap();
        assert!(
            matches!(late, CampaignOutcome::Unreachable(ref p) if p.total_launches == 1),
            "{late:?}"
        );
    }

    #[test]
    fn a_finished_impactor_waits_for_a_better_date() {
        // One built a period from day 0, two launches allowed in any period. A weak
        // window inside the first period, a strong one in the second, a weak one in
        // the third. Spending the first impactor at once reaches 11 at most; keeping
        // it for the strong date reaches 20 with the second one beside it.
        let span = 100.0 * DAY;
        let w = [
            window(10.0, (0.0, 1.0)),
            window(150.0, (0.0, 10.0)),
            window(260.0, (0.0, 1.0)),
        ];
        let line = Stock {
            size: 0,
            build_from: Epoch::from_tdb_seconds_past_j2000(0.0),
            mode: StockMode::Counted,
            built_per_period: 1,
        };
        let p = planned(plan_campaign_stocked(Vector2::zeros(), 20.0, &w, 2, span, &line).unwrap());
        assert_eq!(p.launches, vec![0, 2, 0]);
        // Without the wait - one launch a period - the same target is out of reach.
        let none = Stock {
            built_per_period: UNLIMITED_BUILDS,
            ..line
        };
        let o = plan_campaign_stocked(Vector2::zeros(), 20.0, &w, 1, span, &none).unwrap();
        assert!(matches!(o, CampaignOutcome::Unreachable(_)), "{o:?}");
    }

    /// Every rule a lot-limited plan obeys, as caps on runs of windows in date order
    /// (the rolling window starting at each date, "before the build date", "before
    /// lot k+1") - the independent statement [`crate::interval_packing`] solves.
    fn lot_runs(
        t: &[f64],
        cap: u32,
        window_s: f64,
        stocked: i64,
        per: i64,
        build_from: f64,
    ) -> Vec<crate::interval_packing::RunCap> {
        use crate::interval_packing::RunCap;
        let n = t.len();
        let mut runs = Vec::new();
        for s in 0..n {
            let from = t.partition_point(|&x| x < t[s]);
            let to = t.partition_point(|&x| x < t[s] + window_s - ROLLING_SLACK_S);
            runs.push(RunCap {
                from,
                to,
                cap: i64::from(cap),
            });
        }
        runs.push(RunCap {
            from: 0,
            to: t.partition_point(|&x| x < build_from),
            cap: stocked,
        });
        for k in 0..20 {
            let next = build_from + f64::from(k + 1) * (window_s - ROLLING_SLACK_S);
            runs.push(RunCap {
                from: 0,
                to: t.partition_point(|&x| x < next),
                cap: stocked + per * i64::from(k + 1),
            });
        }
        runs
    }

    #[test]
    fn the_lot_chains_match_an_independent_exact_solver() {
        // Every number of launches, along directions that are not all collinear:
        // the chains' best value against the run-cap packing's, and the same
        // launch counts fitting.
        let span = 100.0 * DAY;
        let mut x = 11u64;
        let mut rnd = move || {
            x = x
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (x >> 33) as f64 / f64::from(1u32 << 31)
        };
        let mut compared = 0;
        let mut bound_by_lots = 0;
        for case in 0..300 {
            let n = 4 + (rnd() * 10.0) as usize;
            let build = (rnd() * 3.0).floor() * 70.0;
            let w: Vec<CampaignWindow> = (0..n)
                .map(|_| {
                    let t = if rnd() < 0.3 {
                        build + 100.0 * (rnd() * 4.0).floor()
                            - if rnd() < 0.5 { 1.0e-4 / DAY } else { 0.0 }
                    } else {
                        rnd() * 450.0
                    };
                    window(t, (rnd() * 200.0 - 50.0, 100.0 + rnd() * 900.0))
                })
                .collect();
            let cap = 1 + (rnd() * 4.0) as u32;
            let per = 1 + (rnd() * f64::from(cap)) as u32;
            let size = (rnd() * 3.0) as u32;
            let u = w[(rnd() * n as f64) as usize].shift_per_launch.normalize();
            let stock = Stock {
                size,
                build_from: Epoch::from_tdb_seconds_past_j2000(build * DAY),
                mode: StockMode::Counted,
                built_per_period: per,
            };
            let chains = LotChains::build(&w, cap, span, u, &stock);
            let free = LotChains::build(
                &w,
                cap,
                span,
                u,
                &Stock {
                    built_per_period: UNLIMITED_BUILDS,
                    ..stock
                },
            );
            // The packing's view: the useful windows in date order.
            let mut idx: Vec<usize> = (0..n)
                .filter(|&i| w[i].shift_per_launch.dot(&u) > 0.0)
                .collect();
            let when = |i: usize| w[i].launch_epoch.tdb_seconds_past_j2000();
            idx.sort_by(|&a, &b| when(a).total_cmp(&when(b)));
            let t: Vec<f64> = idx.iter().map(|&i| when(i)).collect();
            let v: Vec<f64> = idx.iter().map(|&i| w[i].shift_per_launch.dot(&u)).collect();
            let runs = lot_runs(&t, cap, span, i64::from(size), i64::from(per), build * DAY);
            for l in 1..=12usize {
                let mine = chains.best(l, n);
                let theirs = crate::interval_packing::best_packing(&v, &runs, l as u32);
                let tag = format!("case {case} cap={cap} per={per} size={size} l={l}");
                match (&mine, &theirs) {
                    (None, None) => {}
                    (Some((a, launches)), Some(x)) => {
                        let b: f64 = x.iter().zip(&v).map(|(&k, v)| f64::from(k) * v).sum();
                        assert!(
                            (a - b).abs() <= 1e-9 * b.abs().max(1.0),
                            "{tag}: {a} vs {b}"
                        );
                        // The chains' own arrangement obeys every run too.
                        let mut by_pos = vec![0u32; idx.len()];
                        for (p, &i) in idx.iter().enumerate() {
                            by_pos[p] = launches[i];
                        }
                        for r in &runs {
                            let held: i64 =
                                by_pos[r.from..r.to].iter().map(|&k| i64::from(k)).sum();
                            assert!(held <= r.cap, "{tag}: run {r:?} holds {held}");
                        }
                        compared += 1;
                        if free.best(l, n).is_some_and(|(f, _)| f > a + 1e-9) {
                            bound_by_lots += 1;
                        }
                    }
                    _ => panic!("{tag}: chains {mine:?} vs packing {theirs:?}"),
                }
            }
        }
        // Enough fitting cases, and enough where the lots cost something.
        assert!(
            compared > 500 && bound_by_lots > 100,
            "{compared} compared, {bound_by_lots} bound"
        );
    }

    /// Probe, run by hand (`--ignored --nocapture`): what a priced wait costs the
    /// planner - parked launches offered and time per plan, on a synthetic set the
    /// size of the shipping one (~180 windows over 12 years), free wait vs the
    /// shipping 400 km drag rate.
    #[test]
    #[ignore]
    fn probe_priced_wait_planner_cost() {
        let year = 365.25 * DAY;
        let mut seed: u64 = 7;
        let mut next = move || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let w: Vec<CampaignWindow> = (0..180)
            .map(|_| {
                let t = next() * 11.6 * 365.25;
                let a = next() * std::f64::consts::TAU;
                let m = 500.0 + 3_500.0 * next();
                window(t, (m * a.cos(), m * a.sin()))
            })
            .collect();
        let scale: Vec<f64> = (0..w.len()).map(|_| 0.4 + 0.5 * next()).collect();
        // 400 km, the shipping tumbling area: ~31 m/s a year at 310 s.
        let k = 31.0 / (310.0 * 9.806_65) / year;
        for decay in [0.0, k] {
            let all = with_parked_decaying(&w, &scale, 0.0, 11.64 * 365.25, year, decay);
            for cap in [1u32, 2, 6, 12] {
                let t0 = std::time::Instant::now();
                let o = plan_campaign_rolling(Vector2::new(0.0, 0.0), 26_000.0, &all, cap, year)
                    .unwrap();
                println!(
                    "decay {decay:.2e}/s: {} parked launches, cap {cap}: {:.3} s ({})",
                    all.len() - w.len(),
                    t0.elapsed().as_secs_f64(),
                    match o {
                        CampaignOutcome::Planned(p) => format!("{} launches", p.total_launches),
                        CampaignOutcome::Unreachable(_) => "short".into(),
                        CampaignOutcome::AlreadyClear => "clear".into(),
                    }
                );
            }
        }
    }

    // --- A stock already in orbit -----------------------------------------------

    /// The control: stacks in orbit whose windows are the launched ones, pushing the
    /// same, are the ground stock flown outside the cap - the same plan, to the
    /// launch, at every size, cap, production line and target. (The in-orbit stock
    /// differs only in the windows it is handed: what is left of a stack after the
    /// wait and the plane, [`crate::station_keeping`].)
    #[test]
    fn a_stock_in_orbit_on_the_launched_windows_is_the_outside_cap_stock() {
        let w: Vec<CampaignWindow> = [
            (0.0, (300.0, 900.0)),
            (40.0, (-200.0, 700.0)),
            (95.0, (500.0, 300.0)),
            (130.0, (100.0, 1_100.0)),
            (210.0, (-400.0, 600.0)),
            (260.0, (250.0, 800.0)),
            (330.0, (0.0, 950.0)),
        ]
        .iter()
        .map(|&(t, s)| window(t, s))
        .collect();
        let span = 100.0 * DAY;
        let mut compared = 0;
        for size in [0u32, 1, 2, 5] {
            for cap in [1u32, 2, 3] {
                for per in [1u32, 2, UNLIMITED_BUILDS] {
                    for build_day in [0.0, 120.0] {
                        let ground = Stock {
                            size,
                            build_from: Epoch::from_tdb_seconds_past_j2000(build_day * DAY),
                            mode: StockMode::OutsideCap,
                            built_per_period: per,
                        };
                        for target in [800.0, 2_000.0, 3_500.0, 6_000.0] {
                            let b0 = Vector2::new(50.0, -120.0);
                            let a =
                                plan_campaign_stocked(b0, target, &w, cap, span, &ground).unwrap();
                            let o = plan_campaign_orbit_stocked(
                                b0, target, &w, &w, cap, span, size, &ground,
                            )
                            .unwrap();
                            let what = format!(
                                "size {size} cap {cap} per {per} build {build_day} target {target}"
                            );
                            match (a, o) {
                                (CampaignOutcome::AlreadyClear, CampaignOutcome::AlreadyClear) => {}
                                (CampaignOutcome::Planned(x), CampaignOutcome::Planned(y))
                                | (
                                    CampaignOutcome::Unreachable(x),
                                    CampaignOutcome::Unreachable(y),
                                ) => {
                                    assert_eq!(x.total_launches, y.total_launches, "{what}");
                                    assert!(
                                        (x.predicted_b - y.predicted_b).norm() < 1e-9,
                                        "{what}"
                                    );
                                    // Folded back onto one list of windows: the same launches.
                                    let folded: Vec<u32> = (0..w.len())
                                        .map(|i| y.launches[i] + y.launches[w.len() + i])
                                        .collect();
                                    assert_eq!(x.launches, folded, "{what}");
                                    compared += 1;
                                }
                                (x, y) => panic!("{what}: {x:?} vs {y:?}"),
                            }
                        }
                    }
                }
            }
        }
        assert!(compared > 150, "{compared}");
    }

    /// The stacks do not count against the cap: at a cap of one a year, five stacks
    /// leave on one date, and the launched ones still get their own slots.
    #[test]
    fn stacks_in_orbit_do_not_count_against_the_cap() {
        let w = vec![window(0.0, (0.0, 1_000.0)), window(200.0, (0.0, 900.0))];
        let orbit = vec![window(30.0, (0.0, 800.0))];
        let span = 100.0 * DAY;
        let built = Stock {
            size: 0,
            build_from: Epoch::from_tdb_seconds_past_j2000(0.0),
            mode: StockMode::Counted,
            built_per_period: UNLIMITED_BUILDS,
        };
        let plan = |size| {
            plan_campaign_orbit_stocked(
                Vector2::zeros(),
                5_800.0,
                &w,
                &orbit,
                1,
                span,
                size,
                &built,
            )
            .unwrap()
        };
        let p = match plan(5) {
            CampaignOutcome::Planned(p) => p,
            o => panic!("{o:?}"),
        };
        // 1 000 + 900 launched, then 5 x 800 from orbit = 5 900.
        assert_eq!(p.launches, vec![1, 1, 5]);
        assert_eq!(p.total_launches, 7);
        // With no stacks it cannot reach.
        assert!(matches!(plan(0), CampaignOutcome::Unreachable(_)));
    }

    // --- Through a parking orbit -----------------------------------------------

    /// The direct windows plus every parked launch, as one list for the planner.
    fn with_parked(
        w: &[CampaignWindow],
        scale: &[f64],
        earliest_days: f64,
        latest_days: f64,
        period: f64,
    ) -> Vec<CampaignWindow> {
        with_parked_decaying(w, scale, earliest_days, latest_days, period, 0.0)
    }

    /// [`with_parked`] with a cost of waiting, `decay_per_s`.
    fn with_parked_decaying(
        w: &[CampaignWindow],
        scale: &[f64],
        earliest_days: f64,
        latest_days: f64,
        period: f64,
        decay_per_s: f64,
    ) -> Vec<CampaignWindow> {
        let parked = parked_launches(
            w,
            scale,
            Epoch::from_tdb_seconds_past_j2000(earliest_days * DAY),
            &[],
            Epoch::from_tdb_seconds_past_j2000(latest_days * DAY),
            period,
            decay_per_s,
        )
        .unwrap();
        let mut all = w.to_vec();
        all.extend(parked.iter().map(|p| {
            parked_window(
                &w[p.departs_with],
                scale[p.departs_with] * p.wait_factor,
                p.launch_epoch,
                0,
            )
        }));
        all
    }

    /// By hand: a year's best is flown; the same year's second best, dated *after*
    /// it, is what a launch going up between the two can still reach - but only if
    /// a launch can go up there.
    #[test]
    fn best_ahead_keeps_a_later_second_best_a_launch_can_reach() {
        let flown = [(10.0, 9.0), (400.0, 5.0)];
        let pool = [
            (5.0, 8.0),   // 0: before the flown 9 - a launch here reaches the 9
            (50.0, 7.0),  // 1: after the 9, beats everything later
            (60.0, 6.0),  // 2: beats everything later (the 7 is earlier)
            (300.0, 5.0), // 3: only ties the flown 5 at 400
            (300.0, 5.5), // 4: same date, higher
            (500.0, 0.0), // 5: worthless
            (450.0, 1.0), // 6: the only worth after the flown 5
        ];
        // A launch on every date: every window that beats everything after it.
        let every: Vec<f64> = (0..=500).map(f64::from).collect();
        assert_eq!(best_ahead(&every, &flown, &pool), vec![1, 2, 4, 6]);
        // Launches only at 0, 55 and 420: from 0 the flown 9 is best, from 55 the 6,
        // from 420 the 1 - the 7 and the 5.5 are no launch's best.
        assert_eq!(best_ahead(&[0.0, 55.0, 420.0], &flown, &pool), vec![2, 6]);
        // A launch dated exactly on a window can take it.
        assert_eq!(best_ahead(&[300.0], &flown, &pool), vec![4]);
        // After every window: nothing.
        assert!(best_ahead(&[600.0], &flown, &pool).is_empty());
        // Same date and key as a flown window: the flown one already serves.
        assert!(best_ahead(&[0.0], &[(7.0, 3.0)], &[(7.0, 3.0)]).is_empty());
        // Two pool windows tied: the lower index.
        assert_eq!(best_ahead(&[0.0], &[], &[(7.0, 3.0), (7.0, 3.0)]), vec![0]);
    }

    /// The property itself, on random windows and launch dates: for every launch
    /// date the best window at or after it is the same with the pool cut to what
    /// `best_ahead` keeps as with all of it, and every window kept is strictly the
    /// best from some launch date (removing it would lower that launch's best).
    #[test]
    fn best_ahead_loses_no_parked_launch_and_keeps_nothing_spare() {
        let mut s: u64 = 0x2545_f491_4f6c_dd1d;
        let mut next = move |n: u64| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s % n
        };
        let best_from = |t: f64, a: &[(f64, f64)], b: &[(f64, f64)]| {
            a.iter()
                .chain(b)
                .filter(|w| w.0 >= t)
                .map(|w| w.1)
                .fold(0.0_f64, f64::max)
        };
        for _ in 0..400 {
            // Coarse dates and keys so that ties happen.
            let mut draw = |n: u64| -> Vec<(f64, f64)> {
                (0..next(n))
                    .map(|_| (next(12) as f64 * 30.0, next(8) as f64 - 1.0))
                    .collect()
            };
            let flown = draw(5);
            let pool = draw(9);
            let launches: Vec<f64> = (0..next(5))
                .map(|_| next(13) as f64 * 30.0 - 15.0)
                .collect();
            let kept = best_ahead(&launches, &flown, &pool);
            let cut: Vec<(f64, f64)> = kept.iter().map(|&i| pool[i]).collect();
            for &t in &launches {
                assert_eq!(
                    best_from(t, &flown, &cut),
                    best_from(t, &flown, &pool),
                    "launch at {t}: flown {flown:?} pool {pool:?} kept {kept:?}"
                );
            }
            for j in 0..kept.len() {
                let mut without = cut.clone();
                without.remove(j);
                assert!(
                    launches
                        .iter()
                        .any(|&t| best_from(t, &flown, &without) < best_from(t, &flown, &cut)),
                    "pool {} kept but spare: launches {launches:?} flown {flown:?} pool {pool:?} kept {kept:?}",
                    kept[j]
                );
            }
        }
    }

    /// The dates are `earliest` plus whole periods, and each window's date plus one
    /// or more periods, inside `[earliest, stop]`, sorted and each once.
    #[test]
    fn parked_launch_dates_are_the_periods_after_each_window() {
        let d = parked_launch_dates(&[50.0, 30.0, 130.0], 10.0, &[], 260.0, 100.0);
        // 10, 110, 210 from earliest; 150, 250 from 50; 130, 230 from 30; 230 from 130.
        assert_eq!(d, vec![10.0, 110.0, 130.0, 150.0, 210.0, 230.0, 250.0]);
        // A window before `earliest` still seeds the dates a period on.
        assert_eq!(
            parked_launch_dates(&[-50.0], 0.0, &[], 100.0, 100.0),
            vec![0.0, 50.0, 100.0]
        );
        assert!(parked_launch_dates(&[0.0], 0.0, &[], 100.0, 0.0).is_empty());
        // A build date seeds dates at itself and whole periods on, like `earliest`.
        assert_eq!(
            parked_launch_dates(&[], 10.0, &[75.0], 260.0, 100.0),
            vec![10.0, 75.0, 110.0, 175.0, 210.0]
        );
    }

    /// Only the points furthest along some direction survive: an interior point, a
    /// point on an edge, a repeat and a zero shift are all dropped.
    #[test]
    fn the_hull_keeps_only_points_that_win_some_direction() {
        let p = [
            Vector2::new(0.0, 100.0),  // 0: vertex
            Vector2::new(0.0, 50.0),   // 1: on the edge from the origin to 0
            Vector2::new(80.0, 40.0),  // 2: vertex
            Vector2::new(10.0, 20.0),  // 3: inside
            Vector2::new(0.0, 100.0),  // 4: repeat of 0
            Vector2::new(0.0, -300.0), // 5: vertex, the other side
            Vector2::zeros(),          // 6: no push
        ];
        assert_eq!(hull_vertices(&p), vec![0, 2, 5]);
        // Along a ray from the origin only the longest survives.
        assert_eq!(
            hull_vertices(&[Vector2::new(0.0, 1.0), Vector2::new(0.0, 3.0)]),
            vec![1]
        );
        assert!(hull_vertices(&[]).is_empty());
        // Every direction's best is one of the kept points.
        for k in 0..72 {
            let a = f64::from(k) * 5.0_f64.to_radians();
            let u = Vector2::new(a.cos(), a.sin());
            let best = p
                .iter()
                .map(|q| q.dot(&u))
                .fold(f64::NEG_INFINITY, f64::max);
            if best > 0.0 {
                let kept = hull_vertices(&p)
                    .iter()
                    .map(|&i| p[i].dot(&u))
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!((kept - best).abs() < 1e-9, "direction {k}");
            }
        }
    }

    /// The dates: `earliest` plus whole periods, and each window's date plus whole
    /// periods, up to the last departure — and only departures still ahead.
    #[test]
    fn parked_launches_start_on_the_right_dates() {
        let w = [window(0.0, (0.0, 500.0)), window(250.0, (0.0, 300.0))];
        let p = parked_launches(
            &w,
            &[0.5, 0.5],
            Epoch::from_tdb_seconds_past_j2000(-40.0 * DAY),
            &[],
            Epoch::from_tdb_seconds_past_j2000(1_000.0 * DAY),
            100.0 * DAY,
            0.0,
        )
        .unwrap();
        let days: Vec<(i64, usize)> = p
            .iter()
            .map(|q| {
                (
                    (q.launch_epoch.tdb_seconds_past_j2000() / DAY).round() as i64,
                    q.departs_with,
                )
            })
            .collect();
        // -40 can leave through either window (the stronger is the only hull vertex
        // along +ζ), 60/160 from -40, 100/200 from day 0 — all only through day 250.
        assert_eq!(days, vec![(-40, 0), (60, 1), (100, 1), (160, 1), (200, 1)]);
        assert!(parked_launches(
            &w,
            &[0.5],
            Epoch::from_tdb_seconds_past_j2000(0.0),
            &[],
            Epoch::from_tdb_seconds_past_j2000(1.0),
            DAY,
            0.0
        )
        .is_err());
        assert!(parked_launches(
            &w,
            &[0.5, -1.0],
            Epoch::from_tdb_seconds_past_j2000(0.0),
            &[],
            Epoch::from_tdb_seconds_past_j2000(1.0),
            DAY,
            0.0
        )
        .is_err());
    }

    /// Why parking exists: under one launch per 100 days the strong window at day
    /// 150 takes one direct launch, and the weak ones around it are all that is
    /// left — 1 100 at most. Parked on day 0 and leaving on day 150 at 60 % of the
    /// push, a second launch makes it 1 600.
    #[test]
    fn a_parked_launch_takes_a_strong_date_the_cap_would_waste() {
        let w = [
            window(0.0, (0.0, 100.0)),
            window(100.0, (0.0, 100.0)),
            window(150.0, (0.0, 1_000.0)),
            window(200.0, (0.0, 100.0)),
        ];
        let span = 100.0 * DAY;
        match plan_campaign_rolling(Vector2::zeros(), 1_500.0, &w, 1, span).unwrap() {
            CampaignOutcome::Unreachable(best) => {
                assert!((best.predicted_b.y - 1_100.0).abs() < 1e-9, "{best:?}")
            }
            other => panic!("direct only should fall short, got {other:?}"),
        }
        let all = with_parked(&w, &[0.6; 4], 0.0, 300.0, span);
        let p = planned(plan_campaign_rolling(Vector2::zeros(), 1_500.0, &all, 1, span).unwrap());
        assert_eq!(p.total_launches, 2);
        assert!((p.predicted_b.y - 1_600.0).abs() < 1e-9, "{p:?}");
        assert_eq!(p.launches[2], 1, "the direct launch is the strong date");
        assert!(busiest_rolling_count(&launch_list(&all, &p), span) <= 1);
    }

    /// The dates one period after a **direct** launch are needed, not just those
    /// counted from `earliest`: with one launch per 100 days and launches opening on
    /// day 20, the only way to put three launches on the two strong windows is
    /// direct on day 50, parked on day 150 waiting for day 250, and direct on day
    /// 250 — and day 150 is 50 + 100, not 20 + a multiple of 100. Counted from day
    /// 20 alone, the best is 2 200 and the target is out of reach.
    #[test]
    fn a_parked_launch_can_follow_a_direct_one() {
        let w = [window(50.0, (0.0, 1_000.0)), window(250.0, (0.0, 1_000.0))];
        let span = 100.0 * DAY;
        let all = with_parked(&w, &[0.6, 0.6], 20.0, 400.0, span);
        let p = planned(plan_campaign_rolling(Vector2::zeros(), 2_500.0, &all, 1, span).unwrap());
        assert_eq!(p.total_launches, 3);
        assert!((p.predicted_b.y - 2_600.0).abs() < 1e-9, "{p:?}");
        assert_eq!(&p.launches[..2], &[1, 1], "both direct launches fly");
        assert!(busiest_rolling_count(&launch_list(&all, &p), span) <= 1);
    }

    /// Exact against brute force: collinear windows of both signs, a parked scale
    /// per window, and every arrangement of up to four launches on a 10-day grid —
    /// each launch either direct on a window's own date or parked on any grid date
    /// and leaving through any window still ahead — that keeps every rolling
    /// 100-day window within the cap. The grid holds every date the generator
    /// proposes and many it does not, so a missing date would show as a brute-force
    /// count below the planner's.
    #[test]
    fn the_parked_planner_matches_brute_force() {
        parked_planner_matches_brute_force(0.0, &[500.0, 1_300.0, 2_200.0, 2_900.0, 3_600.0]);
    }

    /// The same brute force when waiting costs mass: a stack that waits `d` days
    /// keeps `e^(−k d)` of itself. Two rates - one gentle (a few per cent over the
    /// span), one steep enough that leaving later beats a stronger, earlier window.
    /// The brute force tries every grid date for every parked launch, so it would see
    /// a late date the generator failed to propose.
    #[test]
    fn the_parked_planner_matches_brute_force_when_waiting_costs() {
        // A fine sweep of targets: with waiting priced, plans differ by a few per
        // cent, which coarse targets step over (a first cut with the five targets
        // above passed even with the free-wait dates - mutation-tested).
        let targets: Vec<f64> = (1..=120).map(|k| 30.0 * f64::from(k)).collect();
        for per_day in [2.0e-4, 4.0e-3] {
            parked_planner_matches_brute_force(per_day / DAY, &targets);
        }
    }

    /// The sharp version, for a priced wait: irregular dates, windows where a parked
    /// stack carries more than a direct launch (`scale` > 1, as above `C3` 58 for
    /// the shipping stack), a 2-day grid, up to three launches - on a hand-made set
    /// and on 20 random ones. The brute force takes, per launch count, the furthest
    /// push each way over **every** arrangement on the grid that keeps the cap,
    /// then reads every target off those, so the target sweep is fine for free.
    /// Mutation-tested: with the free-wait dates it fails, and with the "one period
    /// before a direct launch" dates left out it fails - but only once some windows
    /// are direct-only (scale 0): a window that is also a departure already
    /// proposes those dates as "a departure minus whole periods". Likewise the last
    /// launch date's own periods matter only when it falls before a departure, so
    /// every set also runs with launches stopping at day 230.
    #[test]
    fn a_priced_wait_parks_as_late_as_the_chain_allows() {
        let hand: Vec<(f64, f64, f64)> = vec![
            (0.0, 640.0, 0.6),
            (38.0, -910.0, 0.5),
            (72.0, 370.0, 0.7),
            (94.0, 820.0, 0.55),
            (164.0, -150.0, 0.9),
            (248.0, 450.0, 1.4),
        ];
        let mut seed: u64 = 0x9e37_79b9_7f4a_7c15;
        let mut next = move || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut sets = vec![hand];
        for _ in 0..20 {
            let n = 5 + (next() * 3.0) as usize;
            sets.push(
                (0..n)
                    .map(|_| {
                        let t = 2.0 * (next() * 150.0).floor();
                        let mag = 100.0 + 2_900.0 * next();
                        let sign = if next() < 0.75 { 1.0 } else { -1.0 };
                        // A third are direct-only (a steep asymptote, say): those
                        // dates are not departures, so only the "one period before a
                        // direct launch" rule can propose the date before them.
                        let sc = if next() < 0.33 {
                            0.0
                        } else {
                            0.3 + 1.3 * next()
                        };
                        (t, sign * mag, sc)
                    })
                    .collect(),
            );
        }
        let mut compared = 0;
        // Launches up to day 300 (after every window), and up to day 230 - before
        // some departures, so the last launch date is a date of its own.
        for spec in &sets {
            for per_day in [2.0e-3, 1.0e-2] {
                for latest in [300.0, 230.0] {
                    compared += priced_wait_case(spec, per_day, latest);
                }
            }
        }
        assert!(compared > 6_000, "{compared}");
    }

    /// One set of windows `(day, ζ push, parked scale)` at one decay rate (per day),
    /// parked launches going up no later than day `latest`: the planner's count
    /// against the brute force's, over a fine target sweep. Returns how many counts
    /// were compared.
    fn priced_wait_case(spec: &[(f64, f64, f64)], per_day: f64, latest: f64) -> usize {
        let w: Vec<CampaignWindow> = spec.iter().map(|&(t, s, _)| window(t, (0.0, s))).collect();
        let scale: Vec<f64> = spec.iter().map(|s| s.2).collect();
        let span = 100.0 * DAY;
        let grid: Vec<f64> = (0..=150).map(|k| 2.0 * f64::from(k)).collect();
        let all = with_parked_decaying(&w, &scale, 0.0, latest, span, per_day / DAY);
        // One launch on grid date d: its furthest push each way.
        let best: Vec<(f64, f64)> = grid
            .iter()
            .map(|&d| {
                let mut v: Vec<f64> = spec
                    .iter()
                    .filter(|s| s.0 >= d && d <= latest)
                    .map(|s| s.1 * s.2 * (-per_day * (s.0 - d)).exp())
                    .collect();
                v.extend(spec.iter().filter(|s| s.0 == d).map(|s| s.1));
                (
                    v.iter().copied().fold(0.0, f64::max),
                    v.iter().copied().fold(0.0, f64::min),
                )
            })
            .collect();
        let mut compared = 0;
        for cap in [1u32, 2] {
            // reach[k] = (furthest up, furthest down) with k launches.
            let mut reach = [(0.0_f64, 0.0_f64); 4];
            #[allow(clippy::needless_range_loop)] // k is also the tuple's length
            for k in 1..=3usize {
                let mut idx = vec![0usize; k];
                loop {
                    let list: Vec<(f64, u32)> = idx.iter().map(|&i| (grid[i] * DAY, 1)).collect();
                    if busiest_rolling_count(&list, span) <= cap {
                        let up: f64 = idx.iter().map(|&i| best[i].0).sum();
                        let down: f64 = idx.iter().map(|&i| best[i].1).sum();
                        reach[k].0 = reach[k].0.max(up);
                        reach[k].1 = reach[k].1.min(down);
                    }
                    let mut j = k;
                    while j > 0 && idx[j - 1] == grid.len() - 1 {
                        j -= 1;
                    }
                    if j == 0 {
                        break;
                    }
                    idx[j - 1] += 1;
                    for m in j..k {
                        idx[m] = idx[j - 1];
                    }
                }
            }
            for b0 in [0.0_f64, 300.0, -500.0] {
                for t in 1..=150 {
                    let target = 40.0 * f64::from(t);
                    let brute = (0..=3usize)
                        .find(|&k| b0 + reach[k].0 >= target || b0 + reach[k].1 <= -target);
                    let got = plan_campaign_rolling(Vector2::new(0.0, b0), target, &all, cap, span)
                        .unwrap();
                    let what = format!(
                        "{spec:?} k={per_day}/d to {latest} cap={cap} b0={b0} target={target}"
                    );
                    match (brute, got) {
                        (Some(0), CampaignOutcome::AlreadyClear) => {}
                        (Some(n), CampaignOutcome::Planned(p)) => {
                            assert_eq!(p.total_launches as usize, n, "{what}");
                            compared += 1;
                        }
                        (None, CampaignOutcome::Planned(p)) => {
                            assert!(p.total_launches > 3, "{what}: planner {}", p.total_launches)
                        }
                        (None, CampaignOutcome::Unreachable(_)) => {}
                        (brute, got) => panic!("{what}: brute {brute:?} vs {got:?}"),
                    }
                }
            }
        }
        compared
    }

    fn parked_planner_matches_brute_force(decay_per_s: f64, targets: &[f64]) {
        let spec: [(f64, f64, f64); 6] = [
            (0.0, 640.0, 0.6),
            (30.0, -910.0, 0.5),
            (70.0, 370.0, 0.7),
            (90.0, 820.0, 0.55),
            (160.0, -150.0, 0.9),
            (240.0, 450.0, 0.6),
        ];
        let w: Vec<CampaignWindow> = spec.iter().map(|&(t, s, _)| window(t, (0.0, s))).collect();
        let scale: Vec<f64> = spec.iter().map(|s| s.2).collect();
        let span = 100.0 * DAY;
        let all = with_parked_decaying(&w, &scale, 0.0, 300.0, span, decay_per_s);
        // Per grid date, the most positive and most negative push one launch can make.
        let grid: Vec<f64> = (0..=30).map(|k| f64::from(k) * 10.0).collect();
        let options = |d: f64| -> (f64, f64) {
            let mut v: Vec<f64> = spec
                .iter()
                .filter(|s| s.0 >= d)
                .map(|s| s.1 * s.2 * (-decay_per_s * (s.0 - d) * DAY).exp())
                .collect();
            v.extend(spec.iter().filter(|s| s.0 == d).map(|s| s.1));
            (
                v.iter().copied().fold(0.0, f64::max),
                v.iter().copied().fold(0.0, f64::min),
            )
        };
        let best: Vec<(f64, f64)> = grid.iter().map(|&d| options(d)).collect();
        for cap in [1u32, 2] {
            for b0 in [0.0_f64, 400.0, -700.0] {
                for &target in targets {
                    // Collinear: |b0 + Σ| clears the target iff the all-positive or the
                    // all-negative choice does, so two sums per arrangement suffice.
                    let mut brute: Option<u32> = if b0.abs() >= target { Some(0) } else { None };
                    'k: for k in 1..=4usize {
                        if brute.is_some() {
                            break;
                        }
                        let mut idx = vec![0usize; k];
                        loop {
                            let list: Vec<(f64, u32)> =
                                idx.iter().map(|&i| (grid[i] * DAY, 1)).collect();
                            if busiest_rolling_count(&list, span) <= cap {
                                let up: f64 = idx.iter().map(|&i| best[i].0).sum();
                                let down: f64 = idx.iter().map(|&i| best[i].1).sum();
                                if b0 + up >= target || b0 + down <= -target {
                                    brute = Some(k as u32);
                                    break 'k;
                                }
                            }
                            // Next non-decreasing index tuple (a multiset of dates).
                            let mut j = k;
                            while j > 0 && idx[j - 1] == grid.len() - 1 {
                                j -= 1;
                            }
                            if j == 0 {
                                break;
                            }
                            idx[j - 1] += 1;
                            for m in j..k {
                                idx[m] = idx[j - 1];
                            }
                        }
                    }
                    let got = plan_campaign_rolling(Vector2::new(0.0, b0), target, &all, cap, span)
                        .unwrap();
                    if let CampaignOutcome::Planned(p) | CampaignOutcome::Unreachable(p) = &got {
                        assert!(
                            busiest_rolling_count(&launch_list(&all, p), span) <= cap,
                            "k={decay_per_s} cap={cap} b0={b0} target={target}: plan breaks the cap"
                        );
                    }
                    match (brute, got) {
                        (Some(0), CampaignOutcome::AlreadyClear) => {}
                        (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                            p.total_launches, n,
                            "k={decay_per_s} cap={cap} b0={b0} target={target}: planner {} vs brute {n}",
                            p.total_launches
                        ),
                        // Brute force stops at four launches; past that only a planner
                        // count above four is consistent.
                        (None, CampaignOutcome::Planned(p)) => assert!(
                            p.total_launches > 4,
                            "cap={cap} b0={b0} target={target}: planner {} unseen by brute",
                            p.total_launches
                        ),
                        (None, CampaignOutcome::Unreachable(_)) => {}
                        (brute, got) => {
                            panic!("cap={cap} b0={b0} target={target}: brute {brute:?} vs {got:?}")
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn impulses_are_sorted_folded_and_skip_unused_windows() {
        let w = [
            window(50.0, (0.0, 1.0)),
            window(10.0, (0.0, 2.0)),
            window(30.0, (0.0, 3.0)),
        ];
        let imp = campaign_impulses(&w, &[2, 0, 5]);
        assert_eq!(imp.len(), 2);
        assert_eq!(imp[0].0, w[2].arrival_epoch);
        assert_eq!(imp[1].0, w[0].arrival_epoch);
        assert_eq!(imp[0].1, 5.0 * w[2].impulse_per_launch);
        assert_eq!(imp[1].1, 2.0 * w[0].impulse_per_launch);
    }

    #[test]
    fn refuses_bad_input() {
        let w = [window(0.0, (0.0, 1.0))];
        assert!(plan_campaign(Vector2::zeros(), 0.0, &w, 1).is_err());
        assert!(plan_campaign(Vector2::zeros(), 1.0, &w, 0).is_err());
        assert!(plan_campaign(Vector2::new(f64::NAN, 0.0), 1.0, &w, 1).is_err());
    }
}
