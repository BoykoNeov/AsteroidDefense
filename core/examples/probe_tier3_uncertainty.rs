//! Run the Tier-3 covariance mapping against the real field, and check the
//! linearisation it rests on.
//!
//! `uncertainty.rs`'s own tests are kernel-free and validate the *mathematics* —
//! the difference scheme against an exactly-linear map, the probability integral
//! against the Rayleigh closed form, the basis invariance. None of that says the
//! pipeline produces a sane answer on a real 12-year arc, which is a separate
//! claim and the one this probe measures.
//!
//! What it prints, and what would falsify each:
//!
//!   1. The Jacobian, and **three cross-checks on it**. The load-bearing one is
//!      that reducing at a fixed epoch gives the same sensitivity as reducing at
//!      each run's own closest approach — the assumption the whole module rests on,
//!      and the one that would be cheapest to get wrong invisibly. The other two
//!      tie `∂b`, `∂r_p` and `∂v_inf` together through `b² = r_p² + 2μr_p/v_inf²`,
//!      and separate the b-vector's radial motion from its swing around Earth.
//!   2. The mapped ellipse for an invented along-track-dominated covariance. It
//!      must be *elongated*; a near-circular b-plane ellipse from a 20:1 cigar
//!      would mean the along-track direction is not the sensitive one, which
//!      contradicts everything the deflection curve already measured.
//!   3. The impact probability, and the σ-distance that explains it.
//!   4. The ±3σ shell residual — whether `Σ_b = J Σ Jᵀ` is still describing the
//!      encounter at the edge of the covariance, or whether the ellipse is a
//!      fiction and the truth is a banana.
//!
//! A σ sweep at the end shows how the probability responds to how well the orbit
//! is known, which is the actual lesson: the same rock, the same trajectory, and
//! an impact probability that moves across orders of magnitude purely because of
//! how long anyone has been watching it.
//!
//! Requires kernels. ~30 s per covariance for the checked call.
//!
//!   cargo run -p asteroid_core --release --example probe_tier3_uncertainty

use anise::constants::frames::EARTH_J2000;
use asteroid_core::{
    along_track_unit, BPlaneEncounter, EphemerisPerturber, ImpactorConfig, RealFieldScenario,
    StateCovariance, StateVector, FD_STEP_VELOCITY_MS, SAMPLE_CADENCE_DAYS,
    UNCERTAINTY_REDUCTION_LEAD_SECONDS,
};
use nalgebra::Vector6;
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
    println!("build: {:.2} s", t.elapsed().as_secs_f64());

    let ds = scenario.deflection().expect("deflection");
    let seed = ds
        .nominal()
        .state_at(scenario.epoch0())
        .expect("seed at epoch0");
    let nominal = scenario.nominal_hit(&ds).expect("nominal hit");
    println!(
        "nominal encounter: v_inf {:.3} km/s, b {:.1} km, perigee {:.1} km, capture {:.1} km",
        nominal.v_inf / 1e3,
        nominal.impact_parameter / 1e3,
        nominal.perigee / 1e3,
        nominal.capture_radius / 1e3,
    );

    // The invented covariance (see `StateCovariance::synthetic_along_track` — the
    // rock is synthetic, so this is a shape borrowed from reality, not a
    // measurement). 5e-5 m/s along-track, 20:1 over the other two axes, 1 km
    // isotropic on position.
    let cov = StateCovariance::synthetic_along_track(seed, 5.0e-5, 20.0, 1.0e3)
        .expect("non-degenerate seed");

    let t = Instant::now();
    let sens = scenario.bplane_sensitivity().expect("sensitivity");
    println!("\n13-sample Jacobian: {:.1} s", t.elapsed().as_secs_f64());
    println!("  b-plane metres per metre of initial position, per m/s of initial velocity:");
    for row in 0..2 {
        let c: Vec<String> = (0..6)
            .map(|k| format!("{:>12.4e}", sens.jacobian[(row, k)]))
            .collect();
        println!("    [{}]", c.join(" "));
    }
    // --- three cross-checks, replicating the module's own sampling ------------
    //
    // The module reduces at a fixed epoch; `probe_tier3_cost` reduced at each run's
    // own closest approach. Whether those agree is the assumption this whole layer
    // rests on, so it is measured rather than argued. Replicating the sampling here
    // (same cadence, same reduction epoch, same step) gives r_p, v_inf and B for a
    // ±h along-track pair, which is enough for all three checks.
    let t_hat = along_track_unit(seed).expect("along-track unit");
    let ca_epoch = ds
        .nominal_encounter_epoch()
        .expect("scan")
        .expect("nominal CA");
    let t_reduce = ca_epoch.shifted_by_seconds(-UNCERTAINTY_REDUCTION_LEAD_SECONDS);
    let cadence = SAMPLE_CADENCE_DAYS * 86_400.0;
    let span = t_reduce.tdb_seconds_past_j2000() - scenario.epoch0().tdb_seconds_past_j2000();
    let n_snap = ((span / cadence).ceil() + 1.0) as u32;
    let earth_body = EphemerisPerturber::new(Arc::clone(scenario.ephemeris()), EARTH_J2000);
    let reduce = |s: StateVector| -> BPlaneEncounter {
        let clock = scenario
            .propagate_free(scenario.epoch0(), s, cadence, n_snap)
            .expect("propagate");
        let st = clock.state_at(t_reduce).expect("state at reduction epoch");
        let e = earth_body.state_at(t_reduce).expect("earth state");
        BPlaneEncounter::from_relative_state(
            st.position - e.position,
            st.velocity - e.velocity,
            nominal.mu,
            nominal.earth_radius,
        )
        .expect("hyperbolic")
    };
    let h = FD_STEP_VELOCITY_MS;
    let plus = reduce(StateVector::new(seed.position, seed.velocity + h * t_hat));
    let minus = reduce(StateVector::new(seed.position, seed.velocity - h * t_hat));
    let d_rp = (plus.perigee - minus.perigee) / (2.0 * h);
    let d_vinf = (plus.v_inf - minus.v_inf) / (2.0 * h);
    let d_b = (plus.impact_parameter - minus.impact_parameter) / (2.0 * h);

    // (1) Does reducing at a fixed epoch give the same sensitivity as reducing at
    //     each run's own closest approach? This is the module's founding claim.
    const DPERIGEE_DV_ALONG_AT_CA: f64 = -1.618_45e8; // probe_tier3_cost, 10-day cadence
    println!(
        "\n  (1) ∂r_p/∂v_along  fixed-epoch {d_rp:.5e}  vs at-CA {DPERIGEE_DV_ALONG_AT_CA:.5e}  \
         → {:.4}%",
        (d_rp - DPERIGEE_DV_ALONG_AT_CA) / DPERIGEE_DV_ALONG_AT_CA * 100.0
    );

    // (2) b² = r_p² + 2μ r_p / v_inf² ties the three together. The v_inf term is
    //     carried rather than dropped because holding v_inf fixed is exactly the
    //     error the threat-orbit batch was caught by — but here it is measured, and
    //     it turns out to be negligible: an along-track nudge this small moves the
    //     perigee without meaningfully changing the approach speed. Printed anyway,
    //     because "negligible on this scenario" is a measurement and "negligible"
    //     is an assumption.
    let mu = nominal.mu;
    let (r_p, b, v_inf) = (nominal.perigee, nominal.impact_parameter, nominal.v_inf);
    let db_from_rp = (r_p + mu / (v_inf * v_inf)) / b * d_rp;
    let db_from_vinf = -2.0 * mu * r_p / (v_inf * v_inf * v_inf * b) * d_vinf;
    println!(
        "  (2) ∂b/∂v_along    measured {d_b:.5e}  vs {:.5e} from ∂r_p and ∂v_inf  → {:.4}%",
        db_from_rp + db_from_vinf,
        (d_b - (db_from_rp + db_from_vinf)) / (db_from_rp + db_from_vinf) * 100.0
    );
    println!(
        "      (r_p term {db_from_rp:.4e}, v_inf term {db_from_vinf:.4e} — dropping the second \
         would be {:.1}% wrong)",
        (db_from_rp - d_b) / d_b * 100.0
    );

    // (3) The Jacobian's along-track column is a *vector* derivative: B rotates
    //     within the b-plane as well as changing length. Its radial component —
    //     along B̂ — is the one that must equal ∂b/∂v_along; the full norm includes
    //     the tangential swing and is legitimately larger.
    let mut dv = Vector6::zeros();
    for k in 0..3 {
        dv[3 + k] = t_hat[k];
    }
    let col = sens.jacobian * dv;
    let mean = sens.mean();
    let b_hat = mean / mean.norm();
    let radial = col.dot(&b_hat);
    println!(
        "  (3) B̂·∂B/∂v_along  {radial:.5e}  vs ∂b/∂v_along {d_b:.5e}  → {:.4}%",
        (radial - d_b) / d_b * 100.0
    );
    println!(
        "      |∂B/∂v_along| = {:.4e} — {:.2}× the radial part, the rest is B swinging round \
         Earth, not approaching it",
        col.norm(),
        col.norm() / radial.abs()
    );

    let t = Instant::now();
    let (mapped, report) = scenario
        .bplane_uncertainty_checked(&cov, 3.0)
        .expect("covariance maps");
    let elapsed = t.elapsed().as_secs_f64();
    println!("\n25-sample checked mapping: {elapsed:.1} s");

    let (major, minor) = mapped.sigma_axes();
    println!(
        "\nb-plane 1σ ellipse : {:.1} km × {:.1} km   ({:.1}:1)",
        major / 1e3,
        minor / 1e3,
        major / minor
    );
    println!(
        "nominal crossing   : ({:.1}, {:.1}) km, |B| = {:.1} km",
        mapped.mean[0] / 1e3,
        mapped.mean[1] / 1e3,
        mapped.mean.norm() / 1e3
    );
    println!(
        "capture disc       : {:.1} km radius",
        mapped.capture_radius / 1e3
    );
    match mapped.sigma_distance() {
        Some(d) => println!("σ-distance to Earth: {d:.3} σ"),
        None => println!("σ-distance to Earth: (singular covariance)"),
    }
    println!(
        "impact probability : {:.6e}",
        mapped.impact_probability().expect("well-posed")
    );

    println!(
        "\n±3σ linearity: worst residual {:.3}% at shell sample {} of {}",
        report.max_relative_residual * 100.0,
        report.worst_index,
        report.samples.len()
    );
    println!(
        "  worst |predicted − flown| = {:.3} km against a shell reaching {:.1} km",
        report.max_residual / 1e3,
        report.shell_scale / 1e3
    );
    println!(
        "  {}",
        if report.holds_within(0.05) {
            "the linear map still describes the encounter at 3σ — the ellipse is honest"
        } else {
            "the map has bent by 3σ — Σ_b = J Σ Jᵀ is an approximation here, not the truth"
        }
    );
    let worst = report.samples[report.worst_index];
    println!(
        "  worst: predicted ({:.1}, {:.1}) km vs flown ({:.1}, {:.1}) km",
        worst.predicted[0] / 1e3,
        worst.predicted[1] / 1e3,
        worst.flown[0] / 1e3,
        worst.flown[1] / 1e3,
    );

    // How well the orbit is known is the only thing changing here. Same rock, same
    // trajectory, same encounter — the Jacobian is reused, so this sweep is free.
    println!("\nprobability vs how well the orbit is known (same rock, same trajectory):");
    println!(
        "  {:>14}  {:>16}  {:>12}  {:>14}",
        "σ_along (m/s)", "1σ ellipse major", "σ-distance", "P(impact)"
    );
    for sigma in [
        1.0e-5, 1.0e-4, 5.0e-4, 1.0e-3, 5.0e-3, 1.0e-2, 3.0e-2, 1.0e-1,
    ] {
        let c = match StateCovariance::synthetic_along_track(seed, sigma, 20.0, 1.0e3) {
            Some(c) => c,
            None => continue,
        };
        let m = sens.map(&c);
        let (maj, _) = m.sigma_axes();
        println!(
            "  {sigma:>14.1e}  {:>13.1} km  {:>12}  {:>14.6e}",
            maj / 1e3,
            m.sigma_distance()
                .map(|d| format!("{d:.3} σ"))
                .unwrap_or_else(|| "—".into()),
            m.impact_probability().expect("well-posed")
        );
    }
}
