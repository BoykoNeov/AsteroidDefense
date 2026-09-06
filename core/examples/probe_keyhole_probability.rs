//! Tier 3 **at the resonant return**: map the orbit covariance through *both*
//! encounters and read the impact probability of the return the keyhole sets up.
//!
//! Everything Tier 3 has done so far stops at the first encounter. That is the
//! encounter the deflection is aimed at, and for the shipping campaign the answer
//! there is boring by construction — the designed hit has `P = 1` and the
//! deflected miss has `P = 0`. The interesting probability is at the **return**:
//! a keyhole is a place where a *tiny* uncertainty at encounter 1 becomes a large
//! one three years later, which is exactly the regime where a deterministic
//! hit/miss verdict stops being the honest answer.
//!
//! The chaining is in the **propagation**, not in a matrix product: each column
//! of the Jacobian flies a perturbed seed through the flyby and on to the return,
//! so the amplification is measured rather than modelled. See
//! [`asteroid_core::keyhole_target::return_sensitivity`].
//!
//! # Two honesty caveats, both structural
//!
//! - **The covariance is invented.** The shipping rock is synthetic and has no
//!   observation arc, so it has no orbit-determination covariance. The one used
//!   here borrows the *shape* of a real NEO's (dominantly along-track) and says
//!   so; see `StateCovariance::synthetic_along_track`.
//! - **Δv is held fixed.** Every number below is *orbit* uncertainty, never
//!   delivery uncertainty: a real impulse arrives with its own error, and this
//!   probe models none of it. Whether that error would matter is itself one of
//!   the things the `sweep` measures rather than assumes — at the shipping
//!   covariance the answer is **no** (P moves 1.02× across the whole door,
//!   because the uncertainty already dwarfs it), and it becomes yes only once
//!   the orbit is known ~12× better. An earlier version of this doc asserted the
//!   swing unconditionally; the measurement did not support it.
//!
//! # Modes
//!
//! ```text
//!   cargo run -p asteroid_core --release --example probe_keyhole_probability -- <mode> [dv]
//! ```
//!
//! - `check` (2 flights, ~30 s) — does the fixed-epoch reduction reproduce the
//!   flown solution's return? Run this before spending anything else.
//! - `steps` (24 flights, ~6 min) — the finite-difference plateau study **on this
//!   observable**. The shipping steps were measured against encounter 1 and do
//!   not travel; this is what says which ones do.
//! - `gain` (2 flights, ~30 s) — the chained `∂ζ₂/∂ζ₁` against the closed form's
//!   prediction, and the keyhole width that ratio implies.
//! - `probability` (13 flights, ~3 min) — the Jacobian, the return ellipse, and
//!   `P(impact)` at the floor.
//! - `cadence` (13 flights, ~30 s) — the same Jacobian at a 10-day cadence, to
//!   see whether the cadence bias still cancels once a flyby has multiplied it.
//! - `sweep` (13 + 22 flights, ~10 min) — `P(impact)` across the Δv door at
//!   **three** sizes of orbit uncertainty: the picture the whole batch exists to
//!   produce. Mapping a covariance through a fixed Jacobian is free, so the three
//!   regimes cost one set of flights between them.
//!
//! `dv` defaults to **0.216550 m/s**, the refined floor of the shipping rock's
//! 3:4 keyhole measured by `probe_keyhole_return` (2026-09-06). Re-solving it
//! costs four minutes and this probe is not the place to pay that.
//!
//! That default is the **12-iteration** stop, rounded; the sharper floor is
//! `0.2165483096` (1 087 km instead of 1 130). Deliberately not changed: the
//! 1.6e-6 m/s difference is far below what this layer resolves — the impact
//! probability at a keyhole is set by how well the *orbit* is known and comes out
//! flat across the whole door, so re-running to make digits match would be churn.
//! See `probe_keyhole_floor.rs`.
//!
//! Requires kernels.

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, chained_sample, return_sampling_plan, return_sensitivity, BPlaneUncertainty,
    ChainedSample, CircleBranch, FdSteps, ImpactorConfig, KeyholeAiming, KeyholeShotOptions,
    OpikFrame, RealFieldScenario, Resonance, ReturnSamplingPlan, ReturnSensitivity,
    StateCovariance, StateVector, JULIAN_YEAR_S,
};
use nalgebra::Vector2;
use std::time::Instant;

/// The refined floor of the 3:4 keyhole on the shipping rock, m/s — measured by
/// `probe_keyhole_return` on 2026-09-06, not re-derived here.
const DV_FLOOR_M_S: f64 = 0.216_550;

/// The closed-form aim for the same keyhole, m/s — the same probe's *unrefined*
/// shot, whose return sits ~60 000 km out. Used as the far end of the shape test.
const DV_AIM_M_S: f64 = 0.216_438;

/// The invented covariance's along-track velocity sigma, m/s, and the anisotropy
/// and position sigma that go with it — the same three numbers
/// `probe_tier3_uncertainty` uses, so the two probes' answers are comparable.
const SIGMA_ALONG_M_S: f64 = 5.0e-5;

/// Three sizes of orbit uncertainty spanning the regimes the return ellipse can
/// be in, as scale factors on the **whole** covariance — both the along-track
/// velocity σ and the position σ.
///
/// Scaling one block alone is the trap this constant exists to avoid, and it was
/// walked into first: dividing σ_along by 100 moved the return's 1σ from 131 334
/// km to 128 501 km, i.e. not at all. At the return the **position** block
/// dominates — the Jacobian's position columns are ~1.2e5 b-plane metres per
/// metre, so the 1 km position σ contributes ~117 000 km against the velocity
/// block's ~24 500 km. "How well is the orbit known" is a statement about the
/// whole covariance, and only scaling all of it asks that question.
///
/// The factors bracket the crossover measured against the 11 310 km capture
/// disc: 1σ of ~131 000 km (needle ≫ disc), ~11 000 km (needle ≈ disc), and
/// ~1 300 km (needle ≪ disc, the deterministic limit).
const SIGMA_SCALES: [f64; 3] = [1.0, 1.0 / 12.0, 0.01];
const SIGMA_RATIO: f64 = 20.0;
const SIGMA_POSITION_M: f64 = 1.0e3;

/// Finite-difference steps for the **return** observable, as a fraction of the
/// shipping (encounter-1) pair. Justified by the `steps` mode below; see the
/// printed table before changing it.
const RETURN_STEP_SCALE: f64 = 0.01;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args
        .first()
        .cloned()
        .unwrap_or_else(|| "check".to_string())
        .to_lowercase();
    let dv = args
        .get(1)
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(DV_FLOOR_M_S);

    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&ImpactorConfig::default()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e} (kernels required)");
            std::process::exit(1);
        }
    };
    let eph = scenario.ephemeris().clone();
    let ds = scenario.deflection().expect("deflection");
    let nominal = scenario.nominal_hit(&ds).expect("nominal hit");
    let t_ca = ds
        .nominal_encounter_epoch()
        .expect("epoch")
        .expect("an encounter");
    let (r_km, v_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
        .expect("Earth state");
    let frame = OpikFrame::new(
        &nominal,
        r_km * 1e3,
        v_km * 1e3,
        eph.sun_gm_m3_s2().expect("sun GM"),
    )
    .expect("frame");
    let epoch0 = scenario.epoch0();
    let seed = ds.nominal().state_at(epoch0).expect("seed");
    let direction = -along_track_unit(seed).expect("along-track");
    let resonance = Resonance { h: 3, k: 4 };
    let opts = KeyholeShotOptions::default();
    let aiming = KeyholeAiming {
        frame: &frame,
        earth_radius_m: nominal.earth_radius,
        deflection_epoch: epoch0,
        direction,
    };
    println!(
        "scenario built in {:.1} s; {resonance} {:?} branch, retrograde nudge, Δv {dv:.6} m/s",
        t.elapsed().as_secs_f64(),
        CircleBranch::Minus
    );
    println!(
        "  the covariance below is INVENTED (synthetic rock, no observation arc) and Δv is\n  \
         held FIXED — this is orbit uncertainty only, never delivery uncertainty."
    );

    // One flight pins every epoch the rest of the run samples at.
    let t = Instant::now();
    let (plan, shot) = match return_sampling_plan(
        &scenario, &ds, aiming, dv, resonance, &opts,
        86_400.0, // 1-day cadence: what the flown solution was measured at
    ) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("no sampling plan: {e}");
            std::process::exit(1);
        }
    };
    let flown = shot.flown_return.as_ref().expect("a return");
    println!(
        "\nplan (1 flight, {:.0} s):\n  encounter 1 at {}, b {:.0} km\n  return at {} \
         ({:.3} yr later), closest approach {:.0} km\n  reductions fixed at {} and {}",
        t.elapsed().as_secs_f64(),
        shot.first_epoch.as_hifitime(),
        shot.encounter.impact_parameter / 1e3,
        flown.epoch.as_hifitime(),
        flown.years_after_first,
        flown.distance_m / 1e3,
        plan.first_reduce_epoch.as_hifitime(),
        plan.second_reduce_epoch.as_hifitime(),
    );

    match mode.as_str() {
        "check" => check(&scenario, &plan, &shot),
        "steps" => steps(&scenario, &plan),
        "gain" => {
            let opik1 = &frame;
            gain(&scenario, &plan, &shot, opik1, &eph, resonance);
        }
        "probability" => {
            let sens = build_sensitivity(&scenario, &plan, RETURN_STEP_SCALE);
            probability(&sens, &eph, seed);
        }
        "cadence" => cadence(&scenario, &plan),
        "sweep" => {
            let sens = build_sensitivity(&scenario, &plan, RETURN_STEP_SCALE);
            probability(&sens, &eph, seed);
            sweep(&scenario, &plan, &sens, seed, dv);
        }
        other => {
            eprintln!(
                "unknown mode {other:?}; try check | steps | gain | probability | cadence | sweep"
            );
            std::process::exit(2);
        }
    }
}

/// Does reducing at a **fixed** epoch reproduce the flown solution's return?
///
/// It must, or nothing downstream means anything: the whole Jacobian is built
/// out of fixed-epoch reductions, and if that instrument disagrees with the one
/// `probe_keyhole_return` measured the floor with, the two are not describing the
/// same encounter. What is compared is the *impact parameter*, not the closest
/// approach — `b` is the asymptotic quantity that a reduction 12 h early is
/// entitled to reproduce; the geocentric distance at closest approach is smaller
/// by gravitational focusing and is a different number by construction.
fn check(
    scenario: &RealFieldScenario,
    plan: &ReturnSamplingPlan,
    shot: &asteroid_core::KeyholeShot,
) {
    let t = Instant::now();
    let sample = match chained_sample(scenario, plan, plan.seed) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("nominal chained sample failed: {e}");
            std::process::exit(1);
        }
    };
    println!("\ncheck (1 flight, {:.0} s):", t.elapsed().as_secs_f64());
    let own = shot.encounter.impact_parameter;
    let fixed = sample.first.impact_parameter;
    println!(
        "  encounter 1  b: flown-CA {:.1} km vs fixed-epoch {:.1} km  → {:+.3} % \
         ({:+.1} km)",
        own / 1e3,
        fixed / 1e3,
        100.0 * (fixed - own) / own,
        (fixed - own) / 1e3
    );
    let flown = shot.flown_return.as_ref().expect("a return");
    match flown.encounter.as_ref() {
        Some(e2) => {
            let own2 = e2.impact_parameter;
            let fixed2 = sample.second.impact_parameter;
            println!(
                "  return       b: flown-CA {:.1} km vs fixed-epoch {:.1} km  → {:+.3} % \
                 ({:+.1} km)",
                own2 / 1e3,
                fixed2 / 1e3,
                100.0 * (fixed2 - own2) / own2,
                (fixed2 - own2) / 1e3
            );
            println!(
                "  return capture radius {:.0} km at v∞ {:.3} km/s; the flown closest approach \
                 was {:.0} km",
                sample.second.capture_radius / 1e3,
                sample.second.v_inf / 1e3,
                flown.distance_m / 1e3
            );
            println!(
                "  a fixed-epoch reduction is entitled to reproduce b (asymptotic), not the \
                 closest-approach distance (focused). Agreement here is the licence for \
                 everything below."
            );
        }
        None => println!("  return had no b-plane reduction — nothing to compare"),
    }
}

/// The finite-difference plateau study, on the **return** as the observable.
///
/// The shipping steps were chosen to provoke a 10–20 km response at encounter 1.
/// Downstream of a flyby the same step is multiplied by the flyby's gain, so the
/// question this answers is: at which step is the return response of order the
/// return's own capture disc — small enough that the secant is not spanning the
/// feature, large enough that the difference is not propagation round-off?
///
/// Reported per column as the column value itself. A plateau is where halving the
/// step stops moving it.
fn steps(scenario: &RealFieldScenario, plan: &ReturnSamplingPlan) {
    let base = FdSteps::default();
    let scales = [1.0, 0.3, 0.1, 0.03, 0.01, 0.003];
    println!(
        "\nsteps: the return response to a ± pair, per column. shipping steps are \
         {:.1} m and {:.3e} m/s.",
        base.position_m, base.velocity_ms
    );
    println!(
        "  {:>8}  {:>12}  {:>14}  {:>14}  {:>12}",
        "scale", "step", "|Δb₁| (km)", "|Δb₂| (km)", "gain"
    );
    for col in [0_usize, 3_usize] {
        let name = if col < 3 {
            "position rx"
        } else {
            "velocity vx"
        };
        println!("  -- column {col} ({name}) --");
        let mut previous: Option<Vector2<f64>> = None;
        for scale in scales {
            let s = base.scaled(scale);
            let h = if col < 3 { s.position_m } else { s.velocity_ms };
            let plus = match chained_sample(scenario, plan, offset(plan.seed, col, h)) {
                Ok(v) => v,
                Err(e) => {
                    println!("  {scale:>8.3}  sample failed: {e}");
                    continue;
                }
            };
            let minus = match chained_sample(scenario, plan, offset(plan.seed, col, -h)) {
                Ok(v) => v,
                Err(e) => {
                    println!("  {scale:>8.3}  sample failed: {e}");
                    continue;
                }
            };
            let d1 = (plus.first.b_vector - minus.first.b_vector).norm();
            let d2 = (plus.second.b_vector - minus.second.b_vector).norm();
            // The column, in whatever units this block carries: b-plane metres
            // per metre or per m/s, expressed as the 2-vector magnitude.
            let column = Vector2::new(d2 / (2.0 * h), d1 / (2.0 * h));
            let moved =
                previous.map(|p| 100.0 * (column.x - p.x).abs() / p.x.abs().max(f64::MIN_POSITIVE));
            println!(
                "  {scale:>8.3}  {h:>12.4e}  {:>14.1}  {:>14.1}  {:>12.3e}{}",
                d1 / 1e3,
                d2 / 1e3,
                d2 / d1.max(f64::MIN_POSITIVE),
                match moved {
                    Some(m) => format!("   column moved {m:.3} %"),
                    None => String::new(),
                }
            );
            previous = Some(column);
        }
    }
    println!(
        "  read it as: pick the largest scale whose |Δb₂| is of order the return capture \
         radius AND whose column has stopped moving. A step whose |Δb₂| is several capture \
         radii is a secant across the feature being integrated over."
    );
}

/// The chained `∂ζ₂/∂ζ₁` measured against the closed form's prediction — the one
/// independent check this pipeline has, since an invented covariance cannot
/// falsify a Jacobian.
///
/// The closed form: a b-plane displacement `δζ₁` changes the post-encounter
/// semi-major axis by `∂a'/∂ζ₁ · δζ₁` ([`OpikFrame::gradient_semi_major_axis`]),
/// which is a period error `ΔT/T = 1.5 Δa'/a'`, which over the `h`-year return is
/// an arrival slip, which Earth converts to b-plane metres at ~30 km/s. Nothing
/// in that chain is fitted.
///
/// The same ratio **is** the keyhole gain, so Earth's capture disc divided by it
/// is the keyhole width — measured on the flown trajectory rather than
/// linearised from the map.
fn gain(
    scenario: &RealFieldScenario,
    plan: &ReturnSamplingPlan,
    shot: &asteroid_core::KeyholeShot,
    frame1: &OpikFrame,
    eph: &asteroid_core::ephemeris::Ephemeris,
    resonance: Resonance,
) {
    // A velocity perturbation at the return-tuned step: big enough to move ζ₁ by
    // kilometres, small enough that ζ₂ stays inside the return's own scale.
    let h = FdSteps::default().velocity_ms * RETURN_STEP_SCALE;
    let t = Instant::now();
    let plus = chained_sample(scenario, plan, offset(plan.seed, 3, h)).expect("plus sample");
    let minus = chained_sample(scenario, plan, offset(plan.seed, 3, -h)).expect("minus sample");
    println!("\ngain (2 flights, {:.0} s):", t.elapsed().as_secs_f64());

    let frame2 = match return_frame(&plus, eph, plan) {
        Some(f) => f,
        None => {
            eprintln!("  could not build the return's own Öpik frame");
            return;
        }
    };
    let dz1 = frame1.project(&plus.first.b_vector).y - frame1.project(&minus.first.b_vector).y;
    let dz2 = frame2.project(&plus.second.b_vector).y - frame2.project(&minus.second.b_vector).y;
    let measured = dz2 / dz1;

    // The closed form, at the b-point this shot actually flew.
    let grad = frame1.gradient_semi_major_axis(shot.point);
    let a_prime = shot.a_prime_m;
    let v_earth = frame2.v_earth.norm();
    let years = resonance.h as f64;
    let predicted = v_earth * years * JULIAN_YEAR_S * 1.5 * grad.y / a_prime;

    println!(
        "  ∂ζ₂/∂ζ₁ measured {measured:+.4e}   closed form {predicted:+.4e}   ratio {:.3}",
        measured.abs() / predicted.abs().max(f64::MIN_POSITIVE)
    );
    println!(
        "    (from Δζ₁ {:+.1} km → Δζ₂ {:+.1} km on a {h:.3e} m/s pair; ∂a'/∂ζ₁ = {:.4e}, \
         a' = {:.6} AU, V⊕ = {:.3} km/s)",
        dz1 / 1e3,
        dz2 / 1e3,
        grad.y,
        a_prime / asteroid_core::AU_M,
        v_earth / 1e3
    );
    println!(
        "  signs agree: {}. Magnitudes within tens of percent means the chained propagation \
         and the closed form are describing the same flyby; orders apart means one of them is \
         not, and only this comparison can say so.",
        if measured * predicted > 0.0 {
            "yes"
        } else {
            "NO"
        }
    );

    let width = 2.0 * plus.second.capture_radius / measured.abs();
    println!(
        "\n  keyhole width from the flown gain: 2 × {:.0} km of return capture disc ÷ \
         {:.3e} = {:.2} km of ζ₁.",
        plus.second.capture_radius / 1e3,
        measured.abs(),
        width / 1e3
    );
    println!(
        "  This is the width measured on a trajectory that was actually flown, so it is the \
         calibration the linearised map's width has been waiting for — read it against \
         keyhole.rs's closed-form width for this circle."
    );
}

/// Build the return sensitivity at a chosen step scale, reporting the cost.
fn build_sensitivity(
    scenario: &RealFieldScenario,
    plan: &ReturnSamplingPlan,
    scale: f64,
) -> ReturnSensitivity {
    let steps = FdSteps::default().scaled(scale);
    let t = Instant::now();
    let sens = match return_sensitivity(scenario, plan, steps) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("sensitivity failed: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "\n13-flight chained Jacobian at {:.0} % of the shipping steps ({:.2} m, {:.3e} m/s): \
         {:.0} s",
        100.0 * scale,
        steps.position_m,
        steps.velocity_ms,
        t.elapsed().as_secs_f64()
    );
    println!("  return b-plane metres per metre of seed position, per m/s of seed velocity:");
    for row in 0..2 {
        let c: Vec<String> = (0..6)
            .map(|col| format!("{:>12.4e}", sens.jacobian[(row, col)]))
            .collect();
        println!("    [{}]", c.join(" "));
    }
    sens
}

/// The return ellipse and the impact probability at the floor.
fn probability(
    sens: &ReturnSensitivity,
    eph: &asteroid_core::ephemeris::Ephemeris,
    seed: StateVector,
) {
    let cov = StateCovariance::synthetic_along_track(
        seed,
        SIGMA_ALONG_M_S,
        SIGMA_RATIO,
        SIGMA_POSITION_M,
    )
    .expect("non-degenerate seed");
    let u = sens.map(&cov);
    report_probability("floor", &u);

    if let Some(f2) = sens.opik(eph) {
        let p = f2.project(&sens.nominal.second.b_vector);
        println!(
            "  the return in its OWN Öpik frame: ξ₂ {:.0} km (spatial), ζ₂ {:.0} km (timing), \
             |ζ₂|/|ξ₂| = {:.3}",
            p.x / 1e3,
            p.y / 1e3,
            p.y.abs() / p.x.abs().max(f64::MIN_POSITIVE)
        );
        println!(
            "  that split is the convergence gauge, not the probability frame — the \
             probability is invariant under any orthonormal choice of b-plane axes."
        );
    }
}

/// One `P(impact)` line with the numbers that explain it.
fn report_probability(label: &str, u: &BPlaneUncertainty) {
    let (maj, min) = u.sigma_axes();
    let p = u.impact_probability().unwrap_or(f64::NAN);
    println!(
        "  {label:<10} |mean| {:>10.0} km   capture {:>8.0} km   1σ {:>9.0} × {:>7.1} km   \
         {:>6.2} σ   P = {p:.4}",
        u.mean.norm() / 1e3,
        u.capture_radius / 1e3,
        maj / 1e3,
        min / 1e3,
        u.sigma_distance().unwrap_or(f64::NAN),
    );
}

/// The same Jacobian at a 10-day cadence.
///
/// `uncertainty.rs` pins its 10-day cadence on a cancellation argument: both runs
/// of a central difference are flown at the same cadence, so the systematic error
/// is common and drops out. That was measured at a **single** encounter. Here the
/// residue is multiplied by the flyby's gain before it is read, so the argument
/// has to be re-checked rather than inherited — which is what this does.
fn cadence(scenario: &RealFieldScenario, plan: &ReturnSamplingPlan) {
    let steps = FdSteps::default().scaled(RETURN_STEP_SCALE);
    let fine = build_sensitivity(scenario, plan, RETURN_STEP_SCALE);
    let mut coarse_plan = *plan;
    coarse_plan.cadence_seconds = 10.0 * 86_400.0;
    let t = Instant::now();
    let coarse = match return_sensitivity(scenario, &coarse_plan, steps) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("10-day sensitivity failed: {e}");
            return;
        }
    };
    println!(
        "\ncadence: 1-day against 10-day ({:.0} s for the coarse pass)",
        t.elapsed().as_secs_f64()
    );
    println!(
        "  nominal return b: 1-day {:.1} km, 10-day {:.1} km → {:+.2} % ({:+.0} km)",
        fine.nominal.second.impact_parameter / 1e3,
        coarse.nominal.second.impact_parameter / 1e3,
        100.0 * (coarse.nominal.second.impact_parameter - fine.nominal.second.impact_parameter)
            / fine.nominal.second.impact_parameter,
        (coarse.nominal.second.impact_parameter - fine.nominal.second.impact_parameter) / 1e3,
    );
    println!("  per-column magnitude, 1-day vs 10-day:");
    for col in 0..6 {
        let a = Vector2::new(fine.jacobian[(0, col)], fine.jacobian[(1, col)]).norm();
        let b = Vector2::new(coarse.jacobian[(0, col)], coarse.jacobian[(1, col)]).norm();
        println!(
            "    col {col}: {a:>12.4e}  {b:>12.4e}   {:+8.2} %",
            100.0 * (b - a) / a.max(f64::MIN_POSITIVE)
        );
    }
    println!(
        "  a few per cent means the cancellation still holds through the flyby and the coarse \
         cadence is a legitimate 8× saving; tens of per cent means it does not, and the \
         Jacobian has to be flown at the cadence its solution was measured at."
    );
}

/// `P(impact at the return)` across the Δv door, at three sizes of orbit
/// uncertainty — the picture this batch exists to produce, and not the picture it
/// set out to produce.
///
/// The Jacobian is held fixed and only the **mean** is re-flown per point, which
/// is the same split that makes `BPlaneSensitivity` reusable: `J` describes the
/// trajectory family's sensitivity and barely moves across a door 10⁻⁵ m/s wide,
/// while the mean sweeps straight across the capture disc. One flight per point
/// instead of thirteen — and, because mapping a covariance through a Jacobian is
/// free, *three* covariances per flight instead of one.
///
/// Three is the right number because the answer changes character between them.
/// The 1σ ellipse at the return is a **needle**: enormously long in the timing
/// coordinate `ζ₂` and sub-kilometre across it. What matters is the needle's
/// length against Earth's capture disc:
///
/// - **needle ≫ disc** — the shipping σ. Only the sliver of the needle crossing
///   the disc counts, and sliding the mean *along* the needle (which is all Δv
///   can do, because Δv is a timing knob and the needle lies in the timing
///   coordinate) does not change that sliver. P is flat.
/// - **needle ≈ disc** — the crossover, where the door appears as a peak.
/// - **needle ≪ disc** — the deterministic limit: P is 1 inside the door and 0
///   outside it, which is where the hit/miss verdict is the honest answer again.
fn sweep(
    scenario: &RealFieldScenario,
    plan: &ReturnSamplingPlan,
    sens: &ReturnSensitivity,
    seed: StateVector,
    dv_centre: f64,
) {
    let covs: Vec<(f64, StateCovariance)> = SIGMA_SCALES
        .iter()
        .map(|&k| {
            (
                k,
                StateCovariance::synthetic_along_track(
                    seed,
                    SIGMA_ALONG_M_S * k,
                    SIGMA_RATIO,
                    SIGMA_POSITION_M * k,
                )
                .expect("non-degenerate seed"),
            )
        })
        .collect();

    println!("\nthe three uncertainty regimes, at the floor (free — one Jacobian, three Σ):");
    println!(
        "  {:>8}  {:>13}  {:>11}  {:>14}  {:>9}  {:>10}",
        "Σ scale", "σ_along (m/s)", "σ_pos (m)", "1σ major (km)", "vs disc", "P"
    );
    for (k, c) in &covs {
        let u = sens.map(c);
        let (maj, _) = u.sigma_axes();
        println!(
            "  {k:>8.4}  {:>13.2e}  {:>11.1}  {:>14.0}  {:>8.2}×  {:>10.4}",
            SIGMA_ALONG_M_S * k,
            SIGMA_POSITION_M * k,
            maj / 1e3,
            maj / u.capture_radius,
            u.impact_probability().unwrap_or(f64::NAN)
        );
    }

    let dir = plan.impulse / plan.impulse.norm();
    println!("\nsweep: P across the Δv door, one flight per point, three Σ per flight.");
    println!(
        "  {:>12}  {:>12}  {:>12}  {}",
        "Δv (m/s)",
        "Δv − floor",
        "|mean| (km)",
        covs.iter()
            .map(|(k, _)| format!("{:>12}", format!("P @ Σ×{k:.3}")))
            .collect::<Vec<_>>()
            .join("")
    );
    let mut rows: Vec<(f64, f64, Vec<f64>)> = Vec::new();
    let t = Instant::now();
    let point = |dv: f64, note: &str, rows: &mut Vec<(f64, f64, Vec<f64>)>| {
        let mut p = *plan;
        p.impulse = dv * dir;
        let sample = match chained_sample(scenario, &p, plan.seed) {
            Ok(s) => s,
            Err(e) => {
                println!("  {dv:>12.6}  sample failed: {e}");
                return;
            }
        };
        let probs: Vec<f64> = covs
            .iter()
            .map(|(_, c)| {
                BPlaneUncertainty::from_jacobian(&sens.jacobian, c, &sample.second, sens.basis)
                    .impact_probability()
                    .unwrap_or(f64::NAN)
            })
            .collect();
        let mean = sens.basis.project(&sample.second).norm();
        println!(
            "  {dv:>12.6}  {:>12.2e}  {:>12.0}  {}{note}",
            dv - dv_centre,
            mean / 1e3,
            probs
                .iter()
                .map(|p| format!("{p:>12.4}"))
                .collect::<Vec<_>>()
                .join("")
        );
        rows.push((dv, mean, probs));
    };

    for i in 0..21 {
        point(dv_centre + (i as f64 - 10.0) * 5.0e-6, "", &mut rows);
    }
    // The far end: the unrefined closed-form aim, whose return sits ~5 capture
    // radii out. Whether that reads as "far less likely" depends entirely on
    // which of the three columns you are in — which is the finding.
    point(DV_AIM_M_S, "   ← the unrefined closed-form aim", &mut rows);
    println!(
        "  {} flights in {:.0} s",
        rows.len(),
        t.elapsed().as_secs_f64()
    );

    // What the columns actually did, stated as measurement rather than as the
    // story the run was expected to tell.
    println!("\n  contrast across the door, per Σ (max ÷ min over the 21 swept points):");
    for (i, (k, _)) in covs.iter().enumerate() {
        let vals: Vec<f64> = rows
            .iter()
            .take(21)
            .map(|r| r.2[i])
            .filter(|v| v.is_finite())
            .collect();
        let hi = vals.iter().copied().fold(f64::MIN, f64::max);
        let lo = vals.iter().copied().fold(f64::MAX, f64::min);
        // A door that closes completely divides by zero, and "1e39×" is not a
        // number anyone can read. Say what happened instead.
        if lo <= 1.0e-6 {
            println!(
                "    Σ×{k:.4}:  P from {lo:.2e} to {hi:.4}  → the door closes completely at \
                 the edges: a hard yes/no, not a gradient"
            );
        } else {
            println!(
                "    Σ×{k:.4}:  P from {lo:.4} to {hi:.4}  → contrast {:.2}×",
                hi / lo
            );
        }
    }
}

/// The return's own Öpik frame at the plan's reduction epoch.
fn return_frame(
    sample: &ChainedSample,
    eph: &asteroid_core::ephemeris::Ephemeris,
    plan: &ReturnSamplingPlan,
) -> Option<OpikFrame> {
    let (r_km, v_km) = eph
        .state_km_s(
            EARTH_J2000,
            SUN_J2000,
            plan.second_reduce_epoch.as_hifitime(),
        )
        .ok()?;
    let mu_sun = eph.sun_gm_m3_s2().ok()?;
    OpikFrame::new(&sample.second, r_km * 1e3, v_km * 1e3, mu_sun).ok()
}

/// Add `h` to state component `col` (0–2 position, 3–5 velocity) — the same
/// offset `bplane_jacobian` applies, replicated here so the step study can drive
/// one column at a time.
fn offset(mut s: StateVector, col: usize, h: f64) -> StateVector {
    if col < 3 {
        s.position[col] += h;
    } else {
        s.velocity[col - 3] += h;
    }
    s
}
