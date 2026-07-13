//! Commit B1 — the headless Δv-vs-lead-time sweep, the go/no-go gate for the
//! egui viewer (HANDOFF §1, §10 task 10).
//!
//! Builds a designer Earth-impactor over the real DE440 field, verifies the
//! nominal is a genuine hit, then sweeps the required along-track Δv across a
//! range of lead times spanning several heliocentric orbits and prints the
//! curve. Two numbers decide whether the viewer's headline plot exists:
//!
//! 1. **The log-log slope.** The `Δv ∝ 1/lead` law is a multi-orbit asymptotic;
//!    the slope should steepen toward ≈ −1 as leads cross into many-revolution
//!    territory. A flat/shallow slope means the deflection is not changing the
//!    orbital period as it should — a bug, not a curve.
//! 2. **The wall-clock of one sweep.** This dictates the egui app's architecture
//!    (compute-at-startup vs. background thread streaming points).
//!
//! Run with the kernel env vars set, e.g.
//! `ASTEROID_DE_KERNEL=…/de440s.bsp ASTEROID_PLANETARY_CONSTANTS=…/pck11.pca \
//!  cargo run -p viewer --bin curve --release`.

use std::time::Instant;

use viewer::scenario::{CurveFile, ImpactorConfig, RealFieldScenario, DEFAULT_CURVE_JSON};

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let cfg = ImpactorConfig::default();

    println!("Building designer-impactor scenario over the DE440 field…");
    let t_build = Instant::now();
    let sc = RealFieldScenario::build(&cfg)?;
    let build_secs = t_build.elapsed().as_secs_f64();

    let period_years = sc.period_seconds / (365.25 * 86_400.0);
    let leads_available = (sc.impact_epoch().tdb_seconds_past_j2000()
        - sc.epoch0().tdb_seconds_past_j2000())
        / sc.period_seconds;
    println!(
        "  built in {build_secs:.2} s — nominal verified as a HIT.\n\
         \x20 heliocentric a = {:.4e} m ({:.3} AU), period T = {:.3} yr\n\
         \x20 campaign spans {leads_available:.2} orbits of lead\n",
        sc.semi_major_axis_m,
        sc.semi_major_axis_m / 1.495_978_707e11,
        period_years,
    );

    // Leads in orbital periods: from a fraction of an orbit out to many orbits.
    // Log-spaced so the multi-revolution tail (where the 1/lead law lives) is
    // well sampled without a huge point count.
    let leads_periods = [
        0.1, 0.2, 0.35, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0,
    ];
    let target_perigee_m = 2.0e7; // 20 000 km — comfortably above the capture radius

    println!(
        "Sweeping {} lead times (target perigee {:.0} km)…",
        leads_periods.len(),
        target_perigee_m / 1000.0
    );
    let t_sweep = Instant::now();
    let points = sc.sweep(&leads_periods, target_perigee_m)?;
    let sweep_secs = t_sweep.elapsed().as_secs_f64();

    println!(
        "\n  lead[orbits]   lead[yr]    Δv[m/s]      Δv[mm/s]\n\
         \x20 ------------   --------    ---------    --------"
    );
    for p in &points {
        println!(
            "  {:>10.3}   {:>8.3}   {:>10.4e}   {:>8.3}",
            p.lead_periods,
            p.lead_seconds / (365.25 * 86_400.0),
            p.required_dv,
            p.required_dv * 1000.0,
        );
    }

    // Estimate the log-log slope over the multi-orbit tail (lead ≥ 1 period),
    // where the 1/lead asymptotic should dominate. Least-squares on
    // (ln lead, ln Δv); a slope near −1 confirms the thesis.
    let tail: Vec<(f64, f64)> = points
        .iter()
        .filter(|p| p.lead_periods >= 1.0 && p.required_dv > 0.0)
        .map(|p| (p.lead_seconds.ln(), p.required_dv.ln()))
        .collect();
    let slope_tail = least_squares_slope(&tail);

    let all: Vec<(f64, f64)> = points
        .iter()
        .filter(|p| p.required_dv > 0.0)
        .map(|p| (p.lead_seconds.ln(), p.required_dv.ln()))
        .collect();
    let slope_all = least_squares_slope(&all);

    println!(
        "\n  log-log slope (all leads)      : {}\n\
         \x20 log-log slope (lead ≥ 1 orbit) : {}   <- expect → −1 (the 1/lead law)",
        fmt_slope(slope_all),
        fmt_slope(slope_tail),
    );
    println!("\n  sweep wall-clock: {sweep_secs:.2} s for {} points ({:.2} s/point)", points.len(), sweep_secs / points.len() as f64);

    match slope_tail {
        Some(s) if s < -0.6 => println!("\n  GATE: PASS — multi-orbit slope steepens toward −1; the headline curve is real."),
        Some(s) => println!("\n  GATE: INVESTIGATE — tail slope {s:.3} is shallower than expected (deflection may not be moving the period)."),
        None => println!("\n  GATE: INVESTIGATE — not enough multi-orbit points to fit a slope."),
    }

    // Serialise the curve for the egui app. The sweep is a fixed property of the
    // designed scenario, so it is computed once here and cached to disk; the app
    // loads it instantly and never recomputes (there is no config-editing UI).
    let curve = CurveFile {
        semi_major_axis_m: sc.semi_major_axis_m,
        period_seconds: sc.period_seconds,
        max_lead_seconds: sc.impact_epoch().tdb_seconds_past_j2000()
            - sc.epoch0().tdb_seconds_past_j2000(),
        target_perigee_m,
        points,
    };
    let json = serde_json::to_string_pretty(&curve)?;
    std::fs::write(DEFAULT_CURVE_JSON, &json)?;
    let abs = std::fs::canonicalize(DEFAULT_CURVE_JSON)
        .map(|p| p.display().to_string())
        .unwrap_or_else(|_| DEFAULT_CURVE_JSON.to_string());
    println!("\n  wrote {} point(s) to {abs}", curve.points.len());

    Ok(())
}

/// Least-squares slope of `y` on `x` for `(x, y)` samples; `None` if fewer than
/// two points or zero x-variance.
fn least_squares_slope(pts: &[(f64, f64)]) -> Option<f64> {
    let n = pts.len();
    if n < 2 {
        return None;
    }
    let nf = n as f64;
    let mean_x = pts.iter().map(|p| p.0).sum::<f64>() / nf;
    let mean_y = pts.iter().map(|p| p.1).sum::<f64>() / nf;
    let mut sxx = 0.0;
    let mut sxy = 0.0;
    for &(x, y) in pts {
        sxx += (x - mean_x) * (x - mean_x);
        sxy += (x - mean_x) * (y - mean_y);
    }
    if sxx == 0.0 {
        None
    } else {
        Some(sxy / sxx)
    }
}

fn fmt_slope(s: Option<f64>) -> String {
    match s {
        Some(v) => format!("{v:.3}"),
        None => "n/a".to_string(),
    }
}
