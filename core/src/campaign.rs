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
//! `û` — each up to the per-window cap — is the fewest launches that reach a given
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
//! # The per-window cap is a knob, not a fact
//! How many rockets can fly through one launch window is set by pads, production
//! and cadence, and this project has no sourced number for it. So it is a
//! parameter with no default here. Anything that displays a launch count has to
//! display the cap it was counted under.
//!
//! # Every count is a floor
//! The delivered mass is counted *as* impactor mass — no spacecraft bus, no
//! propellant ([`crate::mission`]'s module doc). For "does one launch fail?" that
//! was the safe direction: it fails even with generous mass. For "how many
//! launches?" it is the flattering direction, so every count this module returns
//! is a **lower bound** on the real campaign.

use nalgebra::{Vector2, Vector3};

use crate::epoch::Epoch;

/// One launch window as the planner sees it: what a single launch through it does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CampaignWindow {
    /// Launch epoch — carried for reporting and ordering, not used by the planner.
    pub launch_epoch: Epoch,
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
    /// No direction reaches the target even with every useful window at its cap.
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
/// least `target_b` from Earth's centre (see the module doc for exactly what
/// "fewest" means and when it is exact).
///
/// `target_b` is an **impact parameter** — convert a perigee target with
/// [`OpikFrame::impact_parameter_for_perigee`](crate::keyhole::OpikFrame::impact_parameter_for_perigee)
/// first; a perigee and an impact parameter are not interchangeable.
pub fn plan_campaign(
    nominal_b: Vector2<f64>,
    target_b: f64,
    windows: &[CampaignWindow],
    max_launches_per_window: u32,
) -> Result<CampaignOutcome, CampaignError> {
    if !(target_b.is_finite() && target_b > 0.0) {
        return Err(CampaignError::InvalidInput(format!(
            "target impact parameter must be finite and > 0 (got {target_b})"
        )));
    }
    if max_launches_per_window == 0 {
        return Err(CampaignError::InvalidInput(
            "max_launches_per_window must be at least 1".into(),
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
    if nominal_b.norm() >= target_b {
        return Ok(CampaignOutcome::AlreadyClear);
    }

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

    let mut best_reached: Option<CampaignPlan> = None;
    let mut best_furthest: Option<CampaignPlan> = None;
    for u in directions {
        let (plan, reached) = fill_along(nominal_b, target_b, windows, max_launches_per_window, u);
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

    Ok(match (best_reached, best_furthest) {
        (Some(p), _) => CampaignOutcome::Planned(p),
        (None, Some(p)) => CampaignOutcome::Unreachable(p),
        // No window moves the aim point at all.
        (None, None) => CampaignOutcome::Unreachable(CampaignPlan {
            launches: vec![0; windows.len()],
            total_launches: 0,
            predicted_b: nominal_b,
        }),
    })
}

/// Greedy fill along one direction: largest positive shift along `u` first, one
/// launch at a time, stopping the moment `|B|` clears the target. Returns the plan
/// and whether it reached.
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
    let mut b = nominal_b;
    let mut total = 0u32;
    for (i, _) in order {
        for _ in 0..cap {
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

    fn window(t_days: f64, shift: (f64, f64)) -> CampaignWindow {
        let e = Epoch::from_tdb_seconds_past_j2000(t_days * 86_400.0);
        CampaignWindow {
            launch_epoch: e,
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
