//! Validate the *rotated* branch of the keyhole map, and pin the b-vector sign by
//! measurement rather than by convention.
//!
//! `probe_keyhole_reach` derives `a'(b, B̂)` from the flyby turn and checks it at
//! `δ = 0` — the un-rotated reconstruction `V⊕ + v∞·Ŝ` against the real
//! pre-encounter heliocentric `a`. That check is necessary and it is not
//! sufficient: it exercises `Ŝ` and the frames, but says nothing about the
//! rotation itself. Specifically it cannot see a sign error in
//!
//!   `Ŝ_out = cos δ·Ŝ − sin δ·B̂`
//!
//! because `geometry.rs` deliberately leaves `b_vector`'s **sign** unasserted
//! (only `|B| = b`, `B ⊥ Ŝ` and `B ⊥ ĥ` are pinned), and because the reach probe
//! sweeps the full 2π of directions — which makes the reachable *band* symmetric
//! under `B̂ → −B̂` and hides the flip completely. A flip does not change the band
//! and does not change `|∂a'/∂b|`. What it does is **mirror the locus**, so which
//! directions cross a given resonance, and the `b_res` reported for each, come out
//! wrong. Until this probe passes, "3:4 at 8.53 R⊕" is a candidate, not a result.
//!
//! The measurement, which needs no new machinery:
//!
//!   1. Solve the impulse that raises the perigee to the low end of the 3:4
//!      resonant locus (`required_dv`) — the actual candidate, not a stand-in.
//!   2. Fly it, and reduce the resulting encounter with the shipping geometry, so
//!      the `B̂` under test is the one `geometry.rs` really produces.
//!   3. Predict the post-encounter heliocentric `a` **both ways**, `−B̂` and `+B̂`.
//!   4. Read the truth from the flown trajectory, weeks past the encounter where
//!      Earth's pull is long since negligible.
//!
//! One of the two predictions matches to the `1.3e-4` approximation ceiling and
//! the other misses by orders more. That is the sign, measured — and it validates
//! the rotation's *magnitude* at the same time, which is the part no convention
//! could have supplied. Note the deflection direction is not a free parameter here
//! (`required_dv` targets a perigee along one impulse direction), so this lands
//! *near* the 3:4 locus rather than on it; hitting the resonance needs two knobs
//! and is the targeting step, not this check.
//!
//! Measured on the shipping scenario: `−B̂` lands at `1.518e-4` and `+B̂` at
//! `7.428e-2`, a 489× separation, so the branch in `probe_keyhole_reach` is the
//! right one. Worth noting what the wrong branch predicted — `0.8231 AU` against
//! the 3:4 resonance's `0.8255 AU`. The flip does not produce nonsense; it
//! produces a plausible near-hit on the very resonance being targeted. That is
//! precisely why this is measured rather than reasoned about.
//!
//! It also shows the sign is not cosmetic. `−B̂` *raises* `a'` here (0.854 →
//! 0.889) while `+B̂` lowers it, and the 3:4 resonance sits on the lowering side —
//! so reaching it needs a deflection of the opposite sense to the along-track
//! impulse used here, not merely a larger one.
//!
//! Requires kernels. **~200 s**, nearly all of it the Δv solve: a bracket-and
//! -bisect where every probe is a 12-year propagation (190 s measured).
//!
//!   cargo run -p asteroid_core --release --example probe_keyhole_rotation

use anise::constants::frames::{EARTH_J2000, SSB_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, closest_approach, DvSolveTol, EphemerisPerturber, ImpactorConfig,
    RealFieldScenario, ScanOptions,
};
use nalgebra::Vector3;
use std::time::Instant;

/// 1 AU in metres.
const AU: f64 = 1.495_978_707e11;
/// The gate the shipping scenario uses — mirrored, as in `probe_keyhole_reach`.
const SHIPPING_SCAN_GATE_M: f64 = 5.0e8;
/// The 3:4 resonance: 4 revolutions in 3 years. The candidate the reach probe
/// picked, named as a ratio so the target perigee below is derived rather than
/// transcribed.
const RESONANCE_H: f64 = 3.0;
const RESONANCE_K: f64 = 4.0;
/// How far past the encounter to read the outgoing orbit. At `v∞ ≈ 7.6 km/s` this
/// is ~0.2 AU from Earth, so the osculating heliocentric orbit there *is* the
/// asymptotic outgoing one. Bounded below by what the span actually covers.
const READ_OUT_DAYS: [f64; 4] = [45.0, 30.0, 20.0, 10.0];

fn semi_major(r: &Vector3<f64>, v: &Vector3<f64>, mu: f64) -> f64 {
    1.0 / (2.0 / r.norm() - v.norm_squared() / mu)
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
    let earth = EphemerisPerturber::new(eph.clone(), EARTH_J2000);
    let scan = ScanOptions {
        max_sample_dt: 6.0 * 3600.0,
        time_tol_seconds: 1.0e-3,
        max_distance: Some(SHIPPING_SCAN_GATE_M),
    };
    let ds = scenario.deflection().expect("deflection");
    let nominal = ds
        .nominal_encounter()
        .expect("nominal encounter")
        .expect("a close approach inside the scan gate");
    let mu_earth = nominal.mu;

    // ---- where the 3:4 locus starts, in perigee ------------------------------
    //
    // Re-derived here from the same closed form rather than carried across as a
    // number, so the two probes cannot drift on what "the 3:4 locus" means.
    let a_res = (RESONANCE_H / RESONANCE_K).powf(2.0 / 3.0) * AU;
    let t_ca_nom = ds
        .nominal_encounter_epoch()
        .expect("encounter epoch")
        .expect("an encounter");
    let (r_e_km, v_e_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca_nom.as_hifitime())
        .expect("Earth heliocentric state");
    let (r_e_nom, v_e_nom): (Vector3<f64>, Vector3<f64>) = (r_e_km * 1e3, v_e_km * 1e3);
    let v_inf_nom = nominal.v_inf;
    let s_nom = nominal.s_hat.normalize();

    // Scan b outward along the single direction that gets closest to `a_res`, just
    // to recover a representative locus radius. The reach probe already mapped the
    // whole arc; all this needs is a target worth flying to.
    let basis = asteroid_core::BPlaneBasis::from_encounter(&nominal);
    let mut best: Option<(f64, f64)> = None; // (b, |a' − a_res|)
    for i in 0..72 {
        let theta = std::f64::consts::TAU * (i as f64) / 72.0;
        let b_hat = theta.cos() * basis.e1 + theta.sin() * basis.e2;
        for j in 0..=400 {
            let b = 11_311.3e3 * (60.0f64).powf(j as f64 / 400.0);
            let delta = 2.0 * (mu_earth / (b * v_inf_nom * v_inf_nom)).atan();
            let s_out = delta.cos() * s_nom - delta.sin() * b_hat;
            let a = semi_major(&r_e_nom, &(v_e_nom + v_inf_nom * s_out), mu_sun);
            if a <= 0.0 {
                continue;
            }
            let err = (a - a_res).abs();
            if best.map(|(_, e)| err < e).unwrap_or(true) {
                best = Some((b, err));
            }
        }
    }
    let (b_target, _) = best.expect("some direction approaches the resonance");
    let c = mu_earth / (v_inf_nom * v_inf_nom);
    let target_perigee = -c + (c * c + b_target * b_target).sqrt();
    println!(
        "\n{}:{} resonance a' = {:.6} AU ⇒ locus b ≈ {:.0} km ⇒ target perigee {:.0} km ({:.2} R⊕)",
        RESONANCE_H as u32,
        RESONANCE_K as u32,
        a_res / AU,
        b_target / 1e3,
        target_perigee / 1e3,
        target_perigee / nominal.earth_radius,
    );

    // ---- fly it --------------------------------------------------------------
    let epoch0 = scenario.epoch0();
    let seed = ds.nominal().state_at(epoch0).expect("seed at epoch0");
    let direction = along_track_unit(seed).expect("non-degenerate seed velocity");
    let t = Instant::now();
    let dv_mag = ds
        .required_dv(
            epoch0,
            direction,
            target_perigee,
            DvSolveTol {
                // Loose on purpose: the achieved encounter is *measured* below, so
                // hitting the target exactly buys nothing and costs propagations.
                rel_tol: 1.0e-2,
                ..Default::default()
            },
        )
        .expect("required_dv");
    println!(
        "required Δv along-track at epoch0: {:.6} m/s   [{:.1} s]",
        dv_mag,
        t.elapsed().as_secs_f64()
    );

    let (clock, _) = ds
        .deflected_trajectory(epoch0, dv_mag * direction)
        .expect("deflected trajectory");
    let ca = closest_approach(&clock, &earth, scan)
        .expect("scan the deflected arc")
        .expect("the deflected pass is still inside the gate");
    let enc = ca
        .b_plane(mu_earth, nominal.earth_radius)
        .expect("reduce the deflected encounter");
    println!(
        "\ndeflected encounter @ {}\n  v_inf {:.4} km/s, b {:.1} km, perigee {:.1} km ({:.2} R⊕), \
         hit {}",
        ca.epoch.as_hifitime(),
        enc.v_inf / 1e3,
        enc.impact_parameter / 1e3,
        enc.perigee / 1e3,
        enc.perigee / enc.earth_radius,
        enc.is_hit(),
    );
    if enc.is_hit() {
        println!("  *** still a hit — the Δv solve did not open a miss; nothing to rotate.");
        std::process::exit(1);
    }

    // ---- predict both ways ---------------------------------------------------
    let (r_e_km, v_e_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, ca.epoch.as_hifitime())
        .expect("Earth heliocentric state at the deflected encounter");
    let (r_e, v_e): (Vector3<f64>, Vector3<f64>) = (r_e_km * 1e3, v_e_km * 1e3);
    let s_hat = enc.s_hat.normalize();
    let b_hat = enc.b_vector.normalize();
    let v_inf = enc.v_inf;
    let delta = 2.0 * (mu_earth / (enc.impact_parameter * v_inf * v_inf)).atan();
    let a_in = semi_major(&r_e, &(v_e + v_inf * s_hat), mu_sun);
    let a_minus = semi_major(
        &r_e,
        &(v_e + v_inf * (delta.cos() * s_hat - delta.sin() * b_hat)),
        mu_sun,
    );
    let a_plus = semi_major(
        &r_e,
        &(v_e + v_inf * (delta.cos() * s_hat + delta.sin() * b_hat)),
        mu_sun,
    );

    // ---- read the truth ------------------------------------------------------
    //
    // The propagation frame is SSB-centred and the orbit is Sun-centred, so the
    // Sun's own barycentric motion has to come off before vis-viva is applied.
    let mut truth: Option<(f64, f64)> = None; // (days, a_out)
    for days in READ_OUT_DAYS {
        let t_out = ca.epoch.shifted_by_seconds(days * 86_400.0);
        if let Ok(state) = clock.state_at(t_out) {
            let (rs_km, vs_km) = eph
                .state_km_s(SUN_J2000, SSB_J2000, t_out.as_hifitime())
                .expect("Sun barycentric state");
            let r_h: Vector3<f64> = state.position - rs_km * 1e3;
            let v_h: Vector3<f64> = state.velocity - vs_km * 1e3;
            truth = Some((days, semi_major(&r_h, &v_h, mu_sun)));
            break;
        }
    }
    let (read_days, a_out) = match truth {
        Some(v) => v,
        None => {
            println!("  *** the span does not reach past the encounter — cannot read the outgoing");
            println!("      orbit. Extend span_margin_days.");
            std::process::exit(1);
        }
    };

    let err_minus = (a_minus - a_out).abs() / a_out.abs();
    let err_plus = (a_plus - a_out).abs() / a_out.abs();

    println!(
        "\nturn angle δ = {:.4}° at b = {:.0} km",
        delta.to_degrees(),
        enc.impact_parameter / 1e3
    );
    println!("post-encounter heliocentric a, read {read_days:.0} d after CA:");
    println!("  truth (flown)            : {:.9} AU", a_out / AU);
    println!("  incoming (δ = 0, control): {:.9} AU", a_in / AU);
    println!(
        "  predicted with −B̂        : {:.9} AU   (rel err {err_minus:.3e})",
        a_minus / AU
    );
    println!(
        "  predicted with +B̂        : {:.9} AU   (rel err {err_plus:.3e})",
        a_plus / AU
    );

    // The verdict. A correct branch lands at the approximation ceiling; the wrong
    // one is not marginally worse, it is a different orbit — so requiring a wide
    // separation is a real test and not a tuned threshold.
    const CEILING: f64 = 1.0e-3;
    let (winner, w_err, l_err) = if err_minus < err_plus {
        ("−B̂", err_minus, err_plus)
    } else {
        ("+B̂", err_plus, err_minus)
    };
    println!("\nverdict:");
    if w_err > CEILING {
        println!(
            "  *** BOTH BRANCHES FAIL ({err_minus:.3e}, {err_plus:.3e}). The rotation is wrong \
             in magnitude, not just in sign — a δ, frame, or plane error. The reach probe's \
             per-resonance b_res cannot be trusted."
        );
        std::process::exit(1);
    }
    if l_err < 10.0 * w_err {
        println!(
            "  *** INCONCLUSIVE: the two branches differ by only {:.1}×, so this geometry does \
             not separate them (δ too small, or B̂ nearly ⊥ to what a' responds to). Re-run with \
             a deeper target perigee.",
            l_err / w_err
        );
        std::process::exit(1);
    }
    println!(
        "  {winner} is the outgoing branch — matched to {w_err:.3e} while the other missed by \
         {l_err:.3e} ({:.0}× worse).",
        l_err / w_err
    );
    println!(
        "  So `Ŝ_out = cos δ·Ŝ {} sin δ·B̂` with geometry.rs's own b_vector sign, measured on a \
         real flown flyby rather than adopted. The reach probe's locus is {}.",
        if winner == "−B̂" { "−" } else { "+" },
        if winner == "−B̂" {
            "confirmed as published"
        } else {
            "MIRRORED — its per-resonance b_res values must be recomputed"
        },
    );
}
