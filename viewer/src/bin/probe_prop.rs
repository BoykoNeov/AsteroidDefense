//! Commit B2 pre-flight — time one animation nudge end to end.
//!
//! The `curve` sweep (Commit B1) measured the cost of one *curve point*: a
//! bracket+bisect over many propagations (~57 s/point). That number decided the
//! curve's architecture (precompute + disk cache, never on the main thread).
//!
//! The *animation* is a different beast: one user nudge = one bisect-free
//! forward propagation of the post-deflection arc, a close-approach scan for the
//! new perigee, **and** the ±1.5-day geocentric resampling
//! ([`RealFieldScenario::frame_from`], with a prebuilt scenario + nominal
//! encounter — exactly what the egui worker calls per nudge). Timing that whole
//! call is the number that decides the animation's interaction feel — instant-ish,
//! spinner-gated, or "shorten the interactive default lead so the arc stays short
//! while the headline chart still shows the full sweep." Timing only the raw
//! propagation would miss the `ENCOUNTER_SAMPLES` ephemeris look-ups the resample
//! adds, so we time `frame_from` itself.
//!
//! Cost scales with the arc length (deflection epoch → impact), so we time it at
//! several leads: the full ~12-yr campaign (worst case) down to a half-orbit. Run
//! with the kernel env vars set, e.g.
//! `ASTEROID_DE_KERNEL=…/de440s.bsp ASTEROID_PLANETARY_CONSTANTS=…/pck11.pca \
//!  cargo run -p viewer --bin probe_prop --release`.

use std::time::Instant;

use nalgebra::Vector3;

use asteroid_core::deflection::along_track_unit;
use viewer::scenario::{
    ImpactorConfig, RealFieldScenario, ENCOUNTER_HALF_WINDOW_SECONDS, ENCOUNTER_SAMPLES,
};

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
    println!(
        "  built in {:.2} s (T = {:.3} yr, campaign spans {:.2} orbits)\n",
        t_build.elapsed().as_secs_f64(),
        sc.period_seconds / (365.25 * 86_400.0),
        (sc.impact_epoch().tdb_seconds_past_j2000() - sc.epoch0().tdb_seconds_past_j2000())
            / sc.period_seconds,
    );

    // Build the DeflectionScenario and the nominal encounter ONCE, exactly as the
    // egui worker does — so each timed nudge pays only the short post-deflection
    // arc plus the geocentric resample, not the full-nominal propagation or the
    // full-span nominal scan.
    let ds = sc.deflection()?;
    let nominal_enc = sc.nominal_hit(&ds)?;
    let t_impact = sc.impact_epoch().tdb_seconds_past_j2000();
    let t0 = sc.epoch0().tdb_seconds_past_j2000();
    let max_lead = t_impact - t0;

    // A representative along-track nudge. The cost is set by the arc length and the
    // integrator tolerance, not the impulse magnitude, so any physical value
    // serves — 0.5 m/s is in the band the curve reported.
    let dv_mag = 0.5_f64;

    // Leads in orbital periods, longest (full campaign) first — arc length, hence
    // cost, falls with each. The full campaign is the animation's worst case.
    let leads_periods = [15.0, 8.0, 4.0, 3.0, 2.0, 1.0, 0.5];
    const REPEATS: usize = 3;

    println!(
        "Timing one animation nudge (frame_from: prop + scan + {ENCOUNTER_SAMPLES}-sample \
         geocentric resample) — {REPEATS} reps each:\n\
         \x20 lead[orbits]   lead[yr]    arc[yr]     min[s]    mean[s]    perigee[km]"
    );
    println!("  ------------   --------    -------     ------    -------    -----------");

    for &lp in &leads_periods {
        let mut lead_seconds = lp * sc.period_seconds;
        if lead_seconds > max_lead {
            lead_seconds = max_lead; // clamp to the campaign start
        }
        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-lead_seconds);

        // Along-track direction from the nominal state at the deflection epoch.
        let seed = ds.nominal().state_at(deflection_epoch)?;
        let dir = along_track_unit(seed).ok_or("nominal state has no heading")?;
        let dv: Vector3<f64> = dv_mag * dir;

        let mut best = f64::INFINITY;
        let mut total = 0.0;
        let mut perigee_km = f64::NAN;
        for _ in 0..REPEATS {
            let t = Instant::now();
            let frame = sc.frame_from(
                &ds,
                nominal_enc,
                deflection_epoch,
                dv,
                ENCOUNTER_HALF_WINDOW_SECONDS,
                ENCOUNTER_SAMPLES,
            )?;
            let secs = t.elapsed().as_secs_f64();
            best = best.min(secs);
            total += secs;
            perigee_km = match frame.deflected_perigee {
                Some(p) => p / 1000.0,
                None => f64::INFINITY, // left the scan gate (clean miss)
            };
        }

        println!(
            "  {:>10.2}   {:>8.3}   {:>7.3}   {:>8.3}   {:>8.3}   {:>11.1}",
            lead_seconds / sc.period_seconds,
            lead_seconds / (365.25 * 86_400.0),
            lead_seconds / (365.25 * 86_400.0),
            best,
            total / REPEATS as f64,
            perigee_km,
        );
    }

    println!(
        "\n  Read: min[s] at the app's default lead (~1.5 orbits) is the cost of one\n\
         \x20 nudge as the animation actually pays it. If it is > ~1–2 s, keep the\n\
         \x20 worker thread + spinner (it is there) and/or shorten the interactive\n\
         \x20 default lead (the arc, not the chart) to stay responsive."
    );

    Ok(())
}
