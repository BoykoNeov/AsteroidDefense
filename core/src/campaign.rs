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

/// The most launches inside any one rolling window of `window_s` seconds,
/// counting each window as half-open `[t, t + window_s)` and starting one at
/// every launch — from `(launch epoch as TDB seconds past J2000, launches)` pairs.
pub fn busiest_rolling_count(launches: &[(f64, u32)], window_s: f64) -> u32 {
    launches
        .iter()
        .map(|&(t0, _)| {
            launches
                .iter()
                .filter(|&&(t, _)| t >= t0 && t < t0 + window_s)
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
    // before window k, so a chain can step from it to k.
    let back: Vec<usize> = (0..w)
        .map(|k| t.partition_point(|&tj| tj <= t[k] - window_s))
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
    let mut furthest = empty;
    for (l, &(reach, _)) in split[cap].iter().enumerate().skip(1) {
        if !reach.is_finite() {
            continue;
        }
        let p = plan_for(l);
        if p.predicted_impact_parameter() >= target_b {
            return (p, true);
        }
        furthest = p;
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
