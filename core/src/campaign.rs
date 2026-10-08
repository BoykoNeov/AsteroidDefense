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
/// are the dates generated. Each sits a whole number of periods after its base only
/// to rounding, which [`ROLLING_SLACK_S`] absorbs on both sides of it.
///
/// # Why these departures are enough
/// At one date the planner wants, along its push direction `û`, the departure with
/// the largest shift along `û`. Over every `û` those are exactly the vertices of the
/// convex hull of the candidate shifts (with the origin), so only hull vertices are
/// kept — usually a few per date, where every later window would be dozens.
pub fn parked_launches(
    windows: &[CampaignWindow],
    scale: &[f64],
    earliest: Epoch,
    latest: Epoch,
    period_s: f64,
) -> Result<Vec<ParkedLaunch>, CampaignError> {
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
    let dates = parked_launch_dates(&all_dates, t_lo, t_hi.min(last_out), period_s);

    let mut out = Vec::new();
    for t in dates {
        let ahead: Vec<usize> = departures
            .iter()
            .copied()
            .filter(|&i| when(i) >= t)
            .collect();
        let shifts: Vec<Vector2<f64>> = ahead
            .iter()
            .map(|&i| scale[i] * windows[i].shift_per_launch)
            .collect();
        for k in hull_vertices(&shifts) {
            out.push(ParkedLaunch {
                launch_epoch: Epoch::from_tdb_seconds_past_j2000(t),
                departs_with: ahead[k],
            });
        }
    }
    Ok(out)
}

/// Every date a parked launch can need (see [`parked_launches`] for why): `earliest`
/// and each direct window's date in `window_dates`, plus whole periods - the window
/// dates themselves are direct launches, so a parked launch starts one period after
/// them. Only dates from `earliest` to `stop`, sorted, each once. All TDB s.
///
/// A period that is not finite and > 0 gives no dates.
pub fn parked_launch_dates(window_dates: &[f64], earliest: f64, stop: f64, period_s: f64) -> Vec<f64> {
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

    // --- Through a parking orbit -----------------------------------------------

    /// The direct windows plus every parked launch, as one list for the planner.
    fn with_parked(
        w: &[CampaignWindow],
        scale: &[f64],
        earliest_days: f64,
        latest_days: f64,
        period: f64,
    ) -> Vec<CampaignWindow> {
        let parked = parked_launches(
            w,
            scale,
            Epoch::from_tdb_seconds_past_j2000(earliest_days * DAY),
            Epoch::from_tdb_seconds_past_j2000(latest_days * DAY),
            period,
        )
        .unwrap();
        let mut all = w.to_vec();
        all.extend(
            parked.iter().map(|p| {
                parked_window(&w[p.departs_with], scale[p.departs_with], p.launch_epoch, 0)
            }),
        );
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
            let launches: Vec<f64> = (0..next(5)).map(|_| next(13) as f64 * 30.0 - 15.0).collect();
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
        let d = parked_launch_dates(&[50.0, 30.0, 130.0], 10.0, 260.0, 100.0);
        // 10, 110, 210 from earliest; 150, 250 from 50; 130, 230 from 30; 230 from 130.
        assert_eq!(d, vec![10.0, 110.0, 130.0, 150.0, 210.0, 230.0, 250.0]);
        // A window before `earliest` still seeds the dates a period on.
        assert_eq!(parked_launch_dates(&[-50.0], 0.0, 100.0, 100.0), vec![0.0, 50.0, 100.0]);
        assert!(parked_launch_dates(&[0.0], 0.0, 100.0, 0.0).is_empty());
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
            Epoch::from_tdb_seconds_past_j2000(1_000.0 * DAY),
            100.0 * DAY,
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
            Epoch::from_tdb_seconds_past_j2000(1.0),
            DAY
        )
        .is_err());
        assert!(parked_launches(
            &w,
            &[0.5, -1.0],
            Epoch::from_tdb_seconds_past_j2000(0.0),
            Epoch::from_tdb_seconds_past_j2000(1.0),
            DAY
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
        let all = with_parked(&w, &scale, 0.0, 300.0, span);
        // Per grid date, the most positive and most negative push one launch can make.
        let grid: Vec<f64> = (0..=30).map(|k| f64::from(k) * 10.0).collect();
        let options = |d: f64| -> (f64, f64) {
            let mut v: Vec<f64> = spec
                .iter()
                .filter(|s| s.0 >= d)
                .map(|s| s.1 * s.2)
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
                for target in [500.0, 1_300.0, 2_200.0, 2_900.0, 3_600.0] {
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
                            "cap={cap} b0={b0} target={target}: plan breaks the cap"
                        );
                    }
                    match (brute, got) {
                        (Some(0), CampaignOutcome::AlreadyClear) => {}
                        (Some(n), CampaignOutcome::Planned(p)) => assert_eq!(
                            p.total_launches, n,
                            "cap={cap} b0={b0} target={target}: planner {} vs brute {n}",
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
