//! Price the keyhole batch before designing it: which resonant returns does this
//! rock's encounter geometry actually *reach*, and is the keyhole wide enough to
//! be worth drawing?
//!
//! A keyhole is a small region of the encounter-1 b-plane that leaves the rock on
//! a resonant orbit which brings it back to an impact `k` revolutions later. Two
//! things must be true before any of that is worth building:
//!
//!   1. Some resonance `a' = (h/k)^(2/3) AU` must lie inside the band of
//!      post-encounter semi-major axes this flyby can produce at all.
//!   2. The keyhole must be resolvable — wide enough in `b` to be distinguishable
//!      from the orbit-determination uncertainty already mapped onto that same
//!      b-plane by `uncertainty.rs`.
//!
//! Both fall out of closed-form flyby geometry, with no propagation beyond the
//! nominal arc the scenario already flies. The flyby rotates the Earth-relative
//! velocity through `δ` where `tan(δ/2) = μ⊕/(b·v∞²)`; adding Earth's heliocentric
//! velocity back gives the outgoing heliocentric orbit. That *is* the analytical
//! resonant-return theory, derived from the deflection this project already
//! models rather than transcribed from a paper — which also means it validates
//! itself (see check 1 below).
//!
//! **What this probe can and cannot support.** Taking the encounter position as
//! Earth's own (`r ≈ R⊕ₒᵣᵦ`) costs `η ≈ 7e-5` in heliocentric radius, hence
//! `δa/a ≈ 2η ≈ 1.3e-4`. Over a 7-year return that is `δT/T = 1.5·δa/a ≈ 2e-4`,
//! a timing slip of ~12 h, during which Earth moves ~1.3e6 km — about a hundred
//! capture radii. So:
//!
//!   - absolute `a'` is good enough to answer *"which resonances are in band"*,
//!     and for nothing else;
//!   - it is ~4 orders too coarse to place the return encounter on the capture
//!     disc, and ~7 orders too coarse for a 600 m Apophis-class keyhole. Every
//!     quantitative keyhole claim has to come from the propagator.
//!   - but `∂a'/∂b` **is** trustworthy: the `r ≈ R⊕ₒᵣᵦ` error is common-mode
//!     across neighbouring `b` and cancels to first order in the derivative.
//!
//! That is why the derivative, not the placement, is the deliverable here.
//!
//! What it prints, and what would falsify each:
//!
//!   1. **The round-trip check.** The *un-rotated* reconstruction `V⊕ + v∞·Ŝ` must
//!      reproduce the rock's real pre-encounter heliocentric semi-major axis. This
//!      runs first and gates everything: it catches a frame mix-up, an SSB-versus
//!      -Sun-centred slip, a wrong `Ŝ` sense, or a vis-viva error before any
//!      sweep number is quoted. Agreement is expected at the `1e-4` ceiling above,
//!      so anything at the percent level is a bug, not the approximation.
//!   2. **The reachable band** of `a'`, swept over the full 2π of b-plane
//!      directions and over `b` from the grazing value `b_capture` outward.
//!      `geometry.rs` deliberately leaves the b-vector's *sign* unasserted, so the
//!      sweep covers the whole circle and the band is sign-independent by
//!      construction. Nothing here depends on a convention this project has not
//!      yet pinned.
//!   3. **Per in-band resonance:** where the resonant locus sits in `b`, the
//!      sensitivity `∂a'/∂b` there, and the implied keyhole width — the `Δb` that
//!      slides the return encounter by one capture diameter.
//!   4. **`σ_b` beside it**, from `bplane_sensitivity()` (the covariance-
//!      independent half, ~14 s, reused rather than recomputed). `Δb_keyhole`
//!      against `σ_b` is the go/no-go: a keyhole far narrower than the error
//!      ellipse is a real feature nobody can aim at, and a keyhole far wider than
//!      the ellipse is one the existing Tier-3 machinery can already resolve.
//!   5. **The perigee to dial**, per resonance — inverting
//!      `b² = r_p² + 2μ⊕r_p/v∞²` gives a target the existing `required_dv` solver
//!      can be pointed at, which is what would let the *shipping* rock host the
//!      keyhole instead of a purpose-built one.
//!
//! Requires kernels. ~30 s (one scenario build, one sensitivity solve; the sweep
//! itself is arithmetic).
//!
//!   cargo run -p asteroid_core --release --example probe_keyhole_reach

use anise::constants::frames::{EARTH_J2000, SSB_J2000, SUN_J2000};
use asteroid_core::{
    find_close_approaches, BPlaneBasis, EphemerisPerturber, ImpactorConfig, RealFieldScenario,
    ScanOptions, StateCovariance,
};
use nalgebra::Vector3;
use std::time::Instant;

/// 1 AU in metres (IAU 2012 definition) — the unit resonances are named in.
const AU: f64 = 1.495_978_707e11;
/// Julian year in seconds, the unit `h` counts in `a' = (h/k)^(2/3)`.
const JULIAN_YEAR_S: f64 = 365.25 * 86_400.0;
/// Distance gate for the close-approach census. Without a gate the scan reports
/// every local range-rate minimum over the whole cruise — one per synodic period,
/// most of them tens of millions of km away and irrelevant.
const CENSUS_GATE_AU: f64 = 0.05;
/// How far before the encounter to sample the real pre-encounter orbit for the
/// round-trip check. Far enough out that Earth's pull has not yet bent the
/// heliocentric orbit appreciably, close enough that it is unambiguously the same
/// approach.
const ROUND_TRIP_LEAD_DAYS: f64 = 30.0;
/// Outer edge of the `b` sweep, in capture radii. Beyond this the deflection is
/// small enough that `a'` has converged to the incoming `a` for our purposes.
const B_SWEEP_MAX_CAPTURE_RADII: f64 = 60.0;
/// Angular resolution of the b-plane direction sweep.
const N_THETA: usize = 72;
/// Resonant-return window: returns sooner than this are not really "returns",
/// and later than this are outside any plausible mission horizon.
const RETURN_YEARS_MIN: f64 = 1.5;
const RETURN_YEARS_MAX: f64 = 20.0;

/// Osculating semi-major axis from a two-body state, metres. Negative for a
/// hyperbolic heliocentric orbit, which a deep enough flyby can produce — the
/// caller checks the sign rather than this silently returning nonsense.
fn semi_major(r: &Vector3<f64>, v: &Vector3<f64>, mu: f64) -> f64 {
    1.0 / (2.0 / r.norm() - v.norm_squared() / mu)
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

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

    let eph = scenario.ephemeris().clone();
    let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
    let ds = scenario.deflection().expect("deflection");

    // ---- the encounter census -------------------------------------------------
    //
    // Not decoration. `nominal_encounter_epoch` reduces at the *minimum-distance*
    // approach, and `uncertainty_sampling_plan` anchors the Tier-3 reduction epoch
    // to it. The moment the span is extended past a *deeper* second encounter —
    // which is the entire point of a keyhole — that anchor silently relocates and
    // the existing Jacobian changes meaning without erroring. Knowing how many
    // approaches the current span holds is what sizes that fix.
    let earth = EphemerisPerturber::new(eph.clone(), EARTH_J2000);
    let census = find_close_approaches(
        ds.nominal(),
        &earth,
        ScanOptions {
            max_distance: Some(CENSUS_GATE_AU * AU),
            ..Default::default()
        },
    )
    .expect("close-approach census");
    println!(
        "\nclose approaches within {CENSUS_GATE_AU} AU over the nominal span: {}",
        census.len()
    );
    for (i, ca) in census.iter().enumerate() {
        println!(
            "  [{i}] {}  {:.4} AU ({:.0} km)",
            ca.epoch.as_hifitime(),
            ca.distance / AU,
            ca.distance / 1e3,
        );
    }

    let enc = ds
        .nominal_encounter()
        .expect("nominal encounter")
        .expect("a close approach inside the scan gate");
    let t_ca = ds
        .nominal_encounter_epoch()
        .expect("encounter epoch")
        .expect("a close approach inside the scan gate");
    let v_inf = enc.v_inf;
    let b_cap = enc.capture_radius;
    let mu_earth = enc.mu;
    println!(
        "\nnominal encounter @ {}\n  v_inf {:.4} km/s, b {:.1} km, perigee {:.1} km, \
         capture {:.1} km, hit {}",
        t_ca.as_hifitime(),
        v_inf / 1e3,
        enc.impact_parameter / 1e3,
        enc.perigee / 1e3,
        b_cap / 1e3,
        enc.is_hit(),
    );

    // Earth's heliocentric state at the encounter — the frame the outgoing orbit
    // is expressed in, and the `r ≈ R⊕ₒᵣᵦ` approximation the module doc bounds.
    let (r_e_km, v_e_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
        .expect("Earth heliocentric state at encounter");
    let r_e: Vector3<f64> = r_e_km * 1e3;
    let v_e: Vector3<f64> = v_e_km * 1e3;

    // ---- check 1: the round trip ---------------------------------------------
    //
    // Un-rotated, the reconstruction must give back the orbit the rock is already
    // on. This gates every number below it.
    let s_hat = enc.s_hat.normalize();
    let a_reconstructed = semi_major(&r_e, &(v_e + v_inf * s_hat), mu_sun);

    let t_pre = t_ca.shifted_by_seconds(-ROUND_TRIP_LEAD_DAYS * 86_400.0);
    let pre = ds.nominal().state_at(t_pre).expect("pre-encounter state");
    let (r_sun_km, v_sun_km) = eph
        .state_km_s(SUN_J2000, SSB_J2000, t_pre.as_hifitime())
        .expect("Sun barycentric state");
    // The propagation frame is SSB-centred; the orbit is Sun-centred.
    let r_pre: Vector3<f64> = pre.position - r_sun_km * 1e3;
    let v_pre: Vector3<f64> = pre.velocity - v_sun_km * 1e3;
    let a_actual = semi_major(&r_pre, &v_pre, mu_sun);
    let round_trip_rel = (a_reconstructed - a_actual).abs() / a_actual.abs();

    println!("\nround-trip check (gates everything below):");
    println!(
        "  reconstructed a from V⊕ + v∞·Ŝ : {:.9} AU",
        a_reconstructed / AU
    );
    println!(
        "  actual a, {ROUND_TRIP_LEAD_DAYS:.0} d pre-encounter: {:.9} AU",
        a_actual / AU
    );
    println!("  relative difference           : {round_trip_rel:.3e}");
    if round_trip_rel > 1.0e-3 {
        println!(
            "  *** FAILED — {round_trip_rel:.3e} is far above the {:.0e} approximation \
             ceiling. This is a frame, sign, or vis-viva bug, not the r ≈ R⊕ₒᵣᵦ \
             approximation. Everything below is meaningless.",
            1.3e-4
        );
        std::process::exit(1);
    }
    println!("  ok — at or below the r ≈ R⊕ₒᵣᵦ approximation ceiling (~1.3e-4)");

    // ---- the closed-form outgoing orbit --------------------------------------
    //
    // Full 2π in the b-plane, because `geometry.rs` leaves the b-vector sign
    // unasserted and this probe must not quietly adopt a convention.
    let basis = BPlaneBasis::from_encounter(&enc);
    let a_prime = |b: f64, theta: f64| -> f64 {
        let b_hat = theta.cos() * basis.e1 + theta.sin() * basis.e2;
        // tan(δ/2) = μ⊕/(b·v∞²): the flyby turn angle.
        let delta = 2.0 * (mu_earth / (b * v_inf * v_inf)).atan();
        // The trajectory bends *toward* Earth, i.e. from Ŝ toward −B̂.
        let s_out = delta.cos() * s_hat - delta.sin() * b_hat;
        semi_major(&r_e, &(v_e + v_inf * s_out), mu_sun)
    };

    // ---- check 2: the reachable band ----------------------------------------
    let b_max = B_SWEEP_MAX_CAPTURE_RADII * b_cap;
    let mut a_lo = f64::INFINITY;
    let mut a_hi = f64::NEG_INFINITY;
    let mut hyperbolic = false;
    for i in 0..N_THETA {
        let theta = std::f64::consts::TAU * (i as f64) / (N_THETA as f64);
        // 200 log-spaced radii from the grazing value outward.
        for j in 0..=200 {
            let f = (j as f64) / 200.0;
            let b = b_cap * (b_max / b_cap).powf(f);
            let a = a_prime(b, theta);
            if a <= 0.0 {
                hyperbolic = true;
                continue;
            }
            a_lo = a_lo.min(a);
            a_hi = a_hi.max(a);
        }
    }
    println!(
        "\nreachable post-encounter a' (b from grazing {:.0} km outward, all 2π):",
        b_cap / 1e3
    );
    println!(
        "  {:.6} .. {:.6} AU   (incoming {:.6} AU)",
        a_lo / AU,
        a_hi / AU,
        a_actual / AU
    );
    if hyperbolic {
        println!("  note: some directions eject the rock (a' < 0) — those are excluded");
    }

    // ---- check 3 & 5: resonances in band ------------------------------------
    println!(
        "\nresonant returns in band  (a' = (h/k)^(2/3) AU, return in h yr, \
         {RETURN_YEARS_MIN}..{RETURN_YEARS_MAX} yr):"
    );
    let mut found: Vec<(u32, u32, f64)> = Vec::new();
    for h in 1..=RETURN_YEARS_MAX as u32 {
        for k in 1..=24u32 {
            if gcd(h, k) != 1 {
                continue;
            }
            let years = h as f64;
            if !(RETURN_YEARS_MIN..=RETURN_YEARS_MAX).contains(&years) {
                continue;
            }
            let a_res = (h as f64 / k as f64).powf(2.0 / 3.0) * AU;
            if a_res >= a_lo && a_res <= a_hi {
                found.push((h, k, a_res));
            }
        }
    }
    found.sort_by(|x, y| x.2.partial_cmp(&y.2).unwrap());

    if found.is_empty() {
        println!("  none. This rock's flyby cannot reach a resonance — a purpose-built");
        println!("  threat orbit is required, and the sweep above is its targeting function.");
    }

    // σ_b, for the go/no-go comparison. The sensitivity is the covariance-
    // independent half of the Tier-3 map, so this reuses it rather than
    // recomputing a Jacobian.
    let t = Instant::now();
    let sens = scenario.bplane_sensitivity().expect("bplane sensitivity");
    let seed = ds
        .nominal()
        .state_at(scenario.epoch0())
        .expect("seed at epoch0");
    let cov = StateCovariance::synthetic_along_track(seed, 5.0e-5, 20.0, 1.0e3)
        .expect("non-degenerate seed");
    let unc = sens.map(&cov);
    let (sigma_major, sigma_minor) = unc.sigma_axes();
    println!(
        "\nb-plane uncertainty (invented along-track covariance, as probe_tier3_uncertainty): \
         σ {:.1} × {:.1} km   [{:.1} s]",
        sigma_major / 1e3,
        sigma_minor / 1e3,
        t.elapsed().as_secs_f64(),
    );

    let v_e_speed = v_e.norm();
    for (h, k, a_res) in &found {
        // Locate the resonant locus: for each direction, bisect b for a'(b) = a_res.
        let mut b_hits: Vec<(f64, f64, f64)> = Vec::new(); // (theta, b_res, da'/db)
        for i in 0..N_THETA {
            let theta = std::f64::consts::TAU * (i as f64) / (N_THETA as f64);
            let f = |b: f64| a_prime(b, theta) - a_res;
            let (mut lo, mut hi) = (b_cap, b_max);
            let (flo, fhi) = (f(lo), f(hi));
            if !flo.is_finite() || !fhi.is_finite() || flo * fhi > 0.0 {
                continue; // this direction never crosses the resonance
            }
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if f(mid) * flo > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let b_res = 0.5 * (lo + hi);
            // Central difference on b, at 1e-4 of the radius — the one quantity
            // here the r ≈ R⊕ₒᵣᵦ approximation does *not* corrupt, because its
            // error is common-mode across neighbouring b.
            let step = 1.0e-4 * b_res;
            let dadb = (a_prime(b_res + step, theta) - a_prime(b_res - step, theta)) / (2.0 * step);
            b_hits.push((theta, b_res, dadb));
        }
        if b_hits.is_empty() {
            continue;
        }

        // The keyhole width. A `Δa'` shifts the period by `ΔT/T = 1.5·Δa'/a'`;
        // after the k revolutions that make up the h-year return the arrival slips
        // by `Δt = h·yr·1.5·Δa'/a'`, during which Earth moves `V⊕·Δt`. Calling one
        // capture *diameter* the tolerance gives the Δa' that still returns to the
        // disc — an order-unity definition of "keyhole", stated so it can be
        // argued with rather than assumed.
        let da_tol = 2.0 * b_cap * a_res / (v_e_speed * (*h as f64) * JULIAN_YEAR_S * 1.5);

        let b_lo = b_hits.iter().map(|x| x.1).fold(f64::INFINITY, f64::min);
        let b_hi = b_hits.iter().map(|x| x.1).fold(f64::NEG_INFINITY, f64::max);
        let steepest = b_hits
            .iter()
            .max_by(|x, y| x.2.abs().partial_cmp(&y.2.abs()).unwrap())
            .unwrap();
        let shallowest = b_hits
            .iter()
            .min_by(|x, y| x.2.abs().partial_cmp(&y.2.abs()).unwrap())
            .unwrap();
        let width_tight = da_tol / steepest.2.abs();
        let width_wide = da_tol / shallowest.2.abs();

        // `Δb = Δa'/|∂a'/∂b|` is a *linearisation*, and it diverges where the
        // locus is tangent to a level set of `a'` — the derivative goes to zero
        // there and the quadratic term, not the linear one, sets the width. A
        // resonance whose `|∂a'/∂b|` spans orders of magnitude across directions
        // contains such a tangency, and its wide end is an artifact rather than a
        // measurement. Flag it instead of quoting a number that is not real.
        let tangency = steepest.2.abs() / shallowest.2.abs() > 100.0;

        // The perigee to hand `required_dv` to put the shipping rock here:
        // invert b² = r_p² + 2μ⊕r_p/v∞².
        let c = mu_earth / (v_inf * v_inf);
        let r_p = -c + (c * c + b_lo * b_lo).sqrt();

        println!(
            "\n  {h}:{k} resonance — a' = {:.6} AU, returns in {h} yr after {k} revs",
            a_res / AU
        );
        println!(
            "    locus in b        : {:.0} .. {:.0} km  ({:.2} .. {:.2} capture radii, \
             {}/{N_THETA} directions cross)",
            b_lo / 1e3,
            b_hi / 1e3,
            b_lo / b_cap,
            b_hi / b_cap,
            b_hits.len(),
        );
        println!(
            "    |∂a'/∂b|          : {:.3e} .. {:.3e} AU/km",
            shallowest.2.abs() / AU * 1e3,
            steepest.2.abs() / AU * 1e3,
        );
        println!(
            "    Δa' for one capture diameter at return: {:.3e} AU",
            da_tol / AU
        );
        println!(
            "    keyhole width in b: {:.3} .. {:.3} km   (vs σ_b {:.1} km ⇒ {:.2e} .. {:.2e} σ){}",
            width_tight / 1e3,
            width_wide / 1e3,
            sigma_major / 1e3,
            width_tight / sigma_major,
            width_wide / sigma_major,
            if tangency {
                "  *** NEAR-TANGENCY: ∂a'/∂b spans >100× across directions, so the wide \
                 end is a linearisation artifact, not a width. Quote the tight end only."
            } else {
                ""
            },
        );
        println!(
            "    perigee to dial   : {:.0} km ({:.2} R⊕){}",
            r_p / 1e3,
            r_p / enc.earth_radius,
            if r_p > enc.earth_radius {
                " — a real miss, reachable by required_dv"
            } else {
                " — INSIDE Earth: not a miss, this locus is unreachable by deflection"
            },
        );
    }

    println!(
        "\nReminder: absolute a' above carries ~1.3e-4 from r ≈ R⊕ₒᵣᵦ, which is ~100 capture\n\
         radii of return-encounter placement. The bands and the ∂a'/∂b derivatives are the\n\
         usable output; every quantitative keyhole claim must come from the propagator."
    );
}
