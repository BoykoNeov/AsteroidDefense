//! Measure the marginal cost of one perturbed-state re-fly — the unit Tier-3
//! covariance work is billed in.
//!
//! Tier 3 maps an initial-state covariance to the b-plane, which means flying
//! perturbed seeds. Two designs hang on this number and they are far apart: if a
//! sample costs the ~10 s a `build` does, a 12-column finite-difference STM is two
//! minutes and any Monte Carlo cross-check is hours; if the build cost is mostly
//! kernel load plus the one-off nominal and the *marginal* re-fly is sub-second,
//! both are affordable and the validation can be chosen on merit instead of budget.
//! This project has twice been bitten by an unmeasured per-call cost, so the number
//! goes on the record before the module is designed.
//!
//! What is timed, in the order a Tier-3 caller pays it:
//!   1. `build` — kernel load + the 12-year backward seed design. Paid once.
//!   2. first `deflection()` — the nominal forward propagation, cached after.
//!   3. `nominal_encounter()` — the full-span close-approach scan + reduction.
//!   4. `evaluate(epoch0, δv)` ×N — the marginal sample: re-fly the **whole**
//!      12-year span from a perturbed state, scan, reduce. This is the worst case
//!      on purpose; a covariance at `epoch0` flies the full arc, unlike a planner
//!      nudge late in the campaign.
//!
//! `evaluate` perturbs velocity only, which is not the full 6-vector an STM needs
//! — position columns want a seed offset the public API cannot express today. It
//! is the right *cost* proxy regardless: the work is propagate + scan + reduce, and
//! which component of the seed moved does not change any of the three.
//!
//! Requires kernels.
//!
//!   cargo run -p asteroid_core --release --example probe_tier3_cost

use anise::constants::frames::EARTH_J2000;
use asteroid_core::{
    along_track_unit, closest_approach, EphemerisPerturber, Epoch, ImpactorConfig,
    RealFieldScenario, ScanOptions, StateVector,
};
use nalgebra::Vector3;
use std::sync::Arc;
use std::time::Instant;

fn main() {
    let cfg = ImpactorConfig::default();

    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&cfg) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e}");
            eprintln!("(this probe needs DE440 kernels — see kernels::resolve)");
            std::process::exit(1);
        }
    };
    let t_build = t.elapsed().as_secs_f64();
    println!(
        "build                : {:>8.3} s   (kernel load + {:.0}-yr backward seed design)",
        t_build, cfg.lead_years
    );

    let t = Instant::now();
    let ds = scenario.deflection().expect("deflection");
    let t_defl_first = t.elapsed().as_secs_f64();
    println!(
        "deflection() #1      : {:>8.3} s   (nominal forward propagation, then cached)",
        t_defl_first
    );

    let t = Instant::now();
    let _ds2 = scenario.deflection().expect("deflection");
    println!(
        "deflection() #2      : {:>8.3} s   (clone of the cached nominal)",
        t.elapsed().as_secs_f64()
    );

    let t = Instant::now();
    let nominal = scenario.nominal_hit(&ds).expect("nominal hit");
    let t_scan = t.elapsed().as_secs_f64();
    println!(
        "nominal_encounter()  : {:>8.3} s   (full-span scan + b-plane reduction)",
        t_scan
    );
    println!(
        "                       v_inf {:.3} km/s, b {:.1} km, perigee {:.1} km",
        nominal.v_inf / 1e3,
        nominal.impact_parameter / 1e3,
        nominal.perigee / 1e3
    );

    // The marginal sample. Perturbations spanning the magnitudes an STM column
    // sweep would use, so a cost that varies with step size would show.
    let epoch0: Epoch = scenario.epoch0();
    println!(
        "\nmarginal re-fly from epoch0 (propagate {:.0} yr + scan + reduce):",
        cfg.lead_years
    );
    let mut total = 0.0;
    let mut n = 0usize;
    for (label, dv) in [
        ("δv = 1e-6 m/s", 1.0e-6),
        ("δv = 1e-4 m/s", 1.0e-4),
        ("δv = 1e-2 m/s", 1.0e-2),
        ("δv = 1e-1 m/s", 1.0e-1),
    ] {
        for axis in 0..3 {
            let mut v = Vector3::zeros();
            v[axis] = dv;
            let t = Instant::now();
            let enc = ds.evaluate(epoch0, v).expect("evaluate");
            let dt = t.elapsed().as_secs_f64();
            total += dt;
            n += 1;
            let perigee_km = enc.map(|e| e.perigee / 1e3).unwrap_or(f64::NAN);
            println!(
                "  {label} axis {axis}: {:>7.3} s   perigee {:>12.3} km  (Δ {:>+11.3} km)",
                dt,
                perigee_km,
                perigee_km - nominal.perigee / 1e3
            );
        }
    }
    let mean = total / n as f64;
    println!("\nmean marginal sample : {:>8.3} s over {n} re-flies", mean);
    println!(
        "  12-column central-difference STM  ≈ {:>7.1} s",
        12.0 * mean
    );
    println!(
        "  12-run ±3σ eigenvector shell      ≈ {:>7.1} s",
        12.0 * mean
    );
    println!(
        "  50-sample deterministic check     ≈ {:>7.1} s",
        50.0 * mean
    );
    println!(
        "  1000-sample isotropic Monte Carlo ≈ {:>7.1} min",
        1000.0 * mean / 60.0
    );

    // --- Is that cost physics, or the snapshot cadence? ----------------------
    //
    // `Clock::propagate` calls `step_dense` once per snapshot, so a 1-day cadence
    // restarts the adaptive stepper 4 400 times across the campaign and pays 3
    // extra force evaluations per accepted sub-step to build a dense segment it
    // then stores. The planner needs all of that — it *draws* the arc. A Tier-3
    // sample does not: it wants one b-plane reduction and nothing else, and the
    // close-approach scan interpolates through the dense output at its own
    // `max_sample_dt` (6 h) regardless of how far apart the snapshots are.
    //
    // So the question is whether a coarser cadence buys time without moving the
    // answer. Both halves are printed: a cheaper sample that quietly shifts the
    // perigee is not a cheaper sample.
    let seed = ds.nominal().state_at(epoch0).expect("seed state at epoch0");
    let earth = EphemerisPerturber::new(Arc::clone(scenario.ephemeris()), EARTH_J2000);
    let scan = ScanOptions {
        max_sample_dt: 6.0 * 3600.0,
        time_tol_seconds: 1.0e-3,
        max_distance: Some(5.0e8),
    };
    let mu_earth = nominal.mu;
    let earth_radius = nominal.earth_radius;
    let total_span = cfg.lead_years * 365.25 * 86_400.0 + cfg.span_margin_days * 86_400.0;

    println!(
        "\ncadence sensitivity (same {:.0}-yr span, same scan, no impulse):",
        cfg.lead_years
    );
    println!(
        "  {:>9}  {:>8}  {:>7}  {:>14}  {:>12}",
        "cadence", "time", "snaps", "perigee km", "Δ vs 1-day m"
    );
    let mut baseline: Option<f64> = None;
    for cadence_days in [1.0_f64, 3.0, 10.0, 30.0, 90.0, 180.0] {
        let cadence = cadence_days * 86_400.0;
        let n_snap = (total_span / cadence).ceil().max(1.0) as u32;
        let t = Instant::now();
        let clock = match scenario.propagate_free(epoch0, seed, cadence, n_snap) {
            Ok(c) => c,
            Err(e) => {
                println!("  {cadence_days:>7.0} d  FAILED: {e}");
                continue;
            }
        };
        let ca = closest_approach(&clock, &earth, scan).expect("scan");
        let dt = t.elapsed().as_secs_f64();
        match ca {
            Some(c) => {
                let enc = c.b_plane(mu_earth, earth_radius).expect("b-plane");
                let perigee_km = enc.perigee / 1e3;
                let base = *baseline.get_or_insert(perigee_km);
                println!(
                    "  {cadence_days:>7.0} d  {dt:>7.3} s  {n_snap:>7}  {perigee_km:>14.6}  {:>12.3}",
                    (perigee_km - base) * 1e3
                );
            }
            None => println!("  {cadence_days:>7.0} d  {dt:>7.3} s  {n_snap:>7}  no CA in gate"),
        }
    }

    // --- The number that actually decides the design -------------------------
    //
    // An STM column is a *difference* of two runs flown at the same cadence, so
    // whatever systematic integration error the cadence induces is common to both
    // and cancels to first order. The absolute perigee above is therefore the
    // wrong figure of merit: a cadence that shifts the perigee 118 m may still
    // give a derivative good to a fraction of a percent, and derivatives are all
    // a covariance mapping ever consumes.
    //
    // So: one central-difference column, ∂(perigee)/∂v_x, computed at each
    // cadence and compared against the 1-day answer. If it holds at 30 days, a
    // sample costs half a second and every validation design is affordable.
    // Three columns, not one. `v_x` is a coordinate axis and nothing physically
    // privileges it — but the **along-track** direction is privileged: it carries
    // the largest sensitivity *and* it is where the cadence's own error lives
    // (accumulated timing error around the orbit). A common error cancels cleanly
    // in a symmetric difference where the response is smooth; less cleanly where
    // it is steep. If along-track degrades while `v_x` holds, then `v_x` was the
    // easy direction and the honest cadence is finer. A position column comes too:
    // metres and m/s are different natural scales and a step size that suits one
    // says nothing about the other.
    let h_v = 1.0e-4_f64; // m/s — linear band per the sweep above (0.4% off ×100 scaling)
    let h_r = 1.0e4_f64; // m — the displacement h_v accumulates over the campaign, /4
    let along = along_track_unit(seed).expect("along-track unit");
    let columns: [(&str, Vector3<f64>, Vector3<f64>, f64); 3] = [
        ("v_x      ", Vector3::zeros(), Vector3::x(), h_v),
        ("v_along  ", Vector3::zeros(), along, h_v),
        ("r_x      ", Vector3::x(), Vector3::zeros(), h_r),
    ];

    println!("\ncentral-difference columns ∂(perigee)/∂x, h_v = {h_v:.0e} m/s, h_r = {h_r:.0e} m:");
    println!(
        "  {:>9}  {:>9}  {:>8}  {:>18}  {:>12}",
        "column", "cadence", "2 runs", "d(perigee)/dx", "vs 1-day"
    );
    for (label, dr, dv, h) in columns {
        let mut d_base: Option<f64> = None;
        for cadence_days in [1.0_f64, 3.0, 10.0, 30.0] {
            let cadence = cadence_days * 86_400.0;
            let n_snap = (total_span / cadence).ceil().max(1.0) as u32;
            let t = Instant::now();
            let mut perigees = [0.0_f64; 2];
            let mut ok = true;
            for (k, sign) in [1.0_f64, -1.0].iter().enumerate() {
                let s =
                    StateVector::new(seed.position + sign * h * dr, seed.velocity + sign * h * dv);
                let clock = match scenario.propagate_free(epoch0, s, cadence, n_snap) {
                    Ok(c) => c,
                    Err(_) => {
                        ok = false;
                        break;
                    }
                };
                match closest_approach(&clock, &earth, scan).expect("scan") {
                    Some(c) => {
                        perigees[k] = c.b_plane(mu_earth, earth_radius).expect("b-plane").perigee
                    }
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            let dt = t.elapsed().as_secs_f64();
            if !ok {
                println!("  {label}  {cadence_days:>7.0} d  {dt:>7.3} s  no CA in gate");
                continue;
            }
            let deriv = (perigees[0] - perigees[1]) / (2.0 * h);
            let base = *d_base.get_or_insert(deriv);
            println!(
                "  {label}  {cadence_days:>7.0} d  {dt:>7.3} s  {deriv:>18.6e}  {:>11.4}%",
                (deriv - base) / base * 100.0
            );
        }
    }

    // --- Step-size study: where is each column's working range? --------------
    //
    // The cadence is settled; the finite-difference step is not. Too large and
    // the column measures curvature instead of the derivative; too small and the
    // difference of two nearly-equal perigees is eaten by round-off (and by the
    // close-approach root-finder's own `time_tol_seconds`). The plateau between
    // is the working range, and it has to be found **per column** because a
    // position step is in metres and a velocity step in m/s — nothing carries the
    // scale across.
    //
    // Halving `h` and watching the derivative is the Richardson check: a plateau
    // means converged, a drift means truncation error, ragged jitter means
    // round-off. Run at the settled cadence, since that is where the STM lives.
    const STUDY_CADENCE_DAYS: f64 = 10.0;
    let cadence = STUDY_CADENCE_DAYS * 86_400.0;
    let n_snap = (total_span / cadence).ceil().max(1.0) as u32;
    println!("\nstep-size study at {STUDY_CADENCE_DAYS:.0}-day cadence (halving h):");
    for (label, dr, dv, hs) in [
        (
            "v_along",
            Vector3::zeros(),
            along,
            vec![
                1.0e-3, 5.0e-4, 2.5e-4, 1.25e-4, 6.25e-5, 3.125e-5, 1.5625e-5, 7.8125e-6,
            ],
        ),
        (
            "r_x    ",
            Vector3::x(),
            Vector3::zeros(),
            vec![
                8.0e4, 4.0e4, 2.0e4, 1.0e4, 5.0e3, 2.5e3, 1.25e3, 6.25e2, 3.125e2, 1.5625e2,
                7.8125e1, 3.90625e1,
            ],
        ),
    ] {
        println!(
            "  {:>8}  {:>12}  {:>18}  {:>12}",
            "column", "h", "d(perigee)/dx", "vs previous"
        );
        let mut prev: Option<f64> = None;
        for h in hs {
            let mut perigees = [0.0_f64; 2];
            let mut ok = true;
            for (k, sign) in [1.0_f64, -1.0].iter().enumerate() {
                let s =
                    StateVector::new(seed.position + sign * h * dr, seed.velocity + sign * h * dv);
                let clock = match scenario.propagate_free(epoch0, s, cadence, n_snap) {
                    Ok(c) => c,
                    Err(_) => {
                        ok = false;
                        break;
                    }
                };
                match closest_approach(&clock, &earth, scan).expect("scan") {
                    Some(c) => {
                        perigees[k] = c.b_plane(mu_earth, earth_radius).expect("b-plane").perigee
                    }
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if !ok {
                println!("  {label}  {h:>12.4e}  no CA in gate");
                continue;
            }
            let deriv = (perigees[0] - perigees[1]) / (2.0 * h);
            match prev {
                Some(p) => println!(
                    "  {label}  {h:>12.4e}  {deriv:>18.8e}  {:>11.5}%",
                    (deriv - p) / p * 100.0
                ),
                None => println!("  {label}  {h:>12.4e}  {deriv:>18.8e}  {:>12}", "—"),
            }
            prev = Some(deriv);
        }
    }
}
