//! Fly a keyhole with the propagator: aim the shipping rock at a resonant-return
//! circle, let the real DE440 field carry it through the flyby and on to the
//! return, and refine the impulse until the return miss hits its floor.
//!
//! Everything in `keyhole.rs` is closed-form, and the module says so: absolute
//! placement carries the `r ≈ R⊕ₒᵣᵦ` error (~1.3e-4 in `a'`, ~12 h of timing over a
//! 3-year return, ~1.3e6 km of Earth motion). So the closed form can only *aim*;
//! whether a return actually happens, and how close it comes, has to be flown.
//! Since the targeting batch that whole recipe lives in
//! [`asteroid_core::keyhole_target`] — this probe is now the **driver**, not the
//! implementation, which is what lets it take any resonance rather than the one
//! it was written around.
//!
//!   1. **Aim** — `aim_at_resonance` picks the circle point at the ξ the nudge
//!      naturally lands on, on the requested branch, and converts it to the
//!      perigee `required_dv` targets. Closed-form and instant; an unreachable
//!      resonance or an ξ wider than the circle is refused here, before any
//!      propagation, instead of turning into a NaN four minutes downstream.
//!   2. **Fly through** — the deflection is re-flown, encounter 1 reduced, and
//!      the resonant orbit propagated forward with a wide (0.05 AU) census gate,
//!      because the closed form's timing error is far outside the shipping gate.
//!   3. **Refine** — the return miss is V-shaped in Δv (it is Earth's motion over
//!      the timing slip), so a golden-section search drives it to its floor.
//!
//! **The floor is read in the return's own Öpik frame**, which is the point of
//! doing this in the core rather than in a script: Δv is a *timing* knob and `ζ`
//! is the timing coordinate, so a converged search must push `ζ₂` small and leave
//! the residual in `|ξ₂|` — the spatial offset between the two orbits, which no
//! timing change can remove. A floor sitting in `ζ₂` would be an unconverged
//! search wearing a physics costume, and the scalar return distance looks
//! identical either way.
//!
//! **Measured through this path (2026-09-06, shipping scenario, 3:4 Minus,
//! retrograde), 26 flights in 623 s at 20 iterations:** the closed-form aim gave
//! Δv = 0.2164375000 m/s and a return at 53 841 km; the refined floor is
//! **0.2165483096 m/s → 1 087 km from Earth's centre on 2042-12-31**, 3.00 yr
//! after the 2040-01-01 flyby. Inside Earth: the 3:4 keyhole is an **impact**
//! keyhole. `keyhole_target.rs` re-flies that Δv as a 30 s regression test.
//!
//! **Run this with 20 iterations, not the default 12.** Twelve stops at
//! 0.2165498080 — 1.6e-6 m/s short, a return of 1 130 km — because the budget runs
//! out long before `rel_tol` binds (see below). Both are honest flights; only the
//! wider one is the floor.
//!
//! **The reported Δv window is not a door width.** It is the final bracket, and at
//! the defaults that is arithmetic: `2 · 0.01 · Δv_aim · 0.618ⁿ`, which is 1.34e-5
//! m/s at n = 12 and 2.86e-7 at n = 20 — a 47× change from nothing but the
//! iteration count. This file used to convert 1.3e-5 into "~13 km of b-plane, the
//! same order as the closed form's 24.9 km keyhole width"; that agreement was a
//! coincidence of running 12 iterations. The door the return actually flies
//! through — the Δv span keeping it inside its own focused capture disc — is
//! ~±2e-5 m/s. See `probe_keyhole_floor.rs`.
//!
//! **What the refinement actually does**, in the return's own frame: the aim
//! lands at `ξ₂ 3 549 / ζ₂ −60 185 km` — a miss that is almost entirely *arrival
//! timing* — and the search drives `ζ₂` to **−26.6 km**, a 2 261× fall, while `ξ₂`
//! moves 13 %. The floor is the orbit-to-orbit offset, and this is the run that
//! shows it rather than asserting it.
//!
//! **Never round the floor Δv.** `ζ₂` responds at 5.4e8 km per m/s, so a
//! six-decimal print carries ±271 km of it. Printing `0.216550` and re-flying
//! *that* is a different shot reading ζ₂ ≈ 890 km, and the resulting mismatch
//! against this file's own table looked for two months like a frame or integrator
//! disagreement. Every Δv here is at ten decimals for that reason.
//!
//! Requires kernels. ~4 min (the Δv solve ~230 s, then ~15 re-flies at ~15 s).
//!
//!   cargo run -p asteroid_core --release --example probe_keyhole_return
//!   cargo run -p asteroid_core --release --example probe_keyhole_return -- 5 6 plus pro
//!
//! Arguments (all optional, positional): `h k branch direction iterations`, where
//! `branch` is `minus`|`plus` (which of the two ζ crossings at that ξ),
//! `direction` is `retro`|`pro` (which ξ side the nudge lands on — measured, not
//! chosen), and `iterations` is the golden-section budget (default 12; each one
//! is a flight; the published 3:4 floor above used **20** — `-- 3 4 minus retro 20`).

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, solve_keyhole_return, CircleBranch, DvSolveTol, ImpactorConfig,
    KeyholeAiming, KeyholeRefineTol, KeyholeShot, KeyholeShotOptions, OpikFrame, RealFieldScenario,
    Resonance, AU_M,
};
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let h: u32 = args.first().and_then(|s| s.parse().ok()).unwrap_or(3);
    let k: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(4);
    let branch = match args.get(2).map(|s| s.to_lowercase()) {
        Some(s) if s.starts_with('p') => CircleBranch::Plus,
        _ => CircleBranch::Minus,
    };
    let retrograde =
        !matches!(args.get(3).map(|s| s.to_lowercase()), Some(s) if s.starts_with("pro"));
    // Golden-section iterations. 12 is enough for the 3:4 (a 1 % bracket that
    // never widens); a resonance whose aim is further off needs more, and the
    // ξ₂/ζ₂ split below is what tells you which case you are in.
    let iterations: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(12);
    let resonance = Resonance { h, k };
    println!(
        "target: {resonance}, {:?} branch, {} nudge",
        branch,
        if retrograde { "retrograde" } else { "prograde" }
    );

    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&ImpactorConfig::default()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e}");
            std::process::exit(1);
        }
    };
    println!("build: {:.2} s", t.elapsed().as_secs_f64());

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
    let prograde = along_track_unit(seed).expect("along-track");
    let direction = if retrograde { -prograde } else { prograde };

    // The ξ the nudge lands on: the nominal's own, which a small along-track
    // impulse barely moves. Held fixed; the aim chooses ζ.
    let xi = frame.project(&nominal.b_vector).x;

    let tol = KeyholeRefineTol {
        max_iterations: iterations,
        ..Default::default()
    };

    let t = Instant::now();
    let solution = match solve_keyhole_return(
        &scenario,
        &ds,
        KeyholeAiming {
            frame: &frame,
            earth_radius_m: nominal.earth_radius,
            deflection_epoch: epoch0,
            direction,
        },
        resonance,
        xi,
        branch,
        &KeyholeShotOptions::default(),
        tol,
        DvSolveTol {
            rel_tol: 1.0e-3,
            ..Default::default()
        },
    ) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("keyhole solve failed: {e}");
            std::process::exit(1);
        }
    };
    let elapsed = t.elapsed().as_secs_f64();

    let aim = &solution.aim;
    println!(
        "\n{resonance} target: ξ {:.0} ζ {:.0} km → b {:.0} km, perigee {:.0} km ({:.2} R⊕); \
         resonant a' {:.6} AU",
        aim.target.x / 1e3,
        aim.target.y / 1e3,
        aim.impact_parameter_m / 1e3,
        aim.perigee_m / 1e3,
        aim.perigee_m / nominal.earth_radius,
        aim.circle.a_prime / AU_M
    );
    println!("closed-form aim Δv: {:.10} m/s", solution.aim_dv_m_s);

    report("aim", &solution.aimed, &nominal);
    report("floor", &solution.best, &nominal);
    println!(
        "\n{} flights in {elapsed:.0} s; Δv window at the floor ~{:.2e} m/s of {:.10}",
        solution.flights, solution.dv_window_m_s, solution.best.dv_m_s
    );

    if let Some(r) = solution.best.flown_return.as_ref() {
        println!(
            "return miss {:.0} km = {:.2} capture radii = {:.3} LD",
            r.distance_m / 1e3,
            r.distance_m / nominal.capture_radius,
            r.distance_m / 384_400e3
        );
        // A search that never bracketed its minimum has stopped on the edge of its
        // own interval, and golden-section reports a *vanishing* Δv window while
        // doing it — so a wall is indistinguishable from a tight answer unless the
        // bracketing is reported. Say it before anything else, because every
        // number underneath it is a number from the wall.
        if !solution.bracketed {
            println!(
                "!! THE SEARCH NEVER BRACKETED THE MINIMUM. Δv {:.10} is the edge of the \
                 interval, not its bottom: the widening reaches {:.0}% either side of \
                 the aim and the minimum is further out. Raise `max_widenings`; the \
                 Δv window below is the wall closing, not convergence.",
                solution.best.dv_m_s,
                100.0 * tol.reach_fraction()
            );
        }
        // Whether the residual is a *floor* is a measurement, not a conclusion of
        // having stopped searching. Δv buys arrival time, so a converged minimum
        // has spent the timing coordinate ζ₂ and left the spatial one ξ₂. If ζ₂ still
        // dominates, the search ran out of iterations and calling what is left
        // "the orbit-to-orbit offset" is simply false — which is exactly what this
        // probe printed for 7:9 before the split was checked.
        let timing = r.timing_share().unwrap_or(f64::NAN);
        let converged = timing < 1.0;
        if solution.is_impact_return() {
            println!(
                "** RESONANT-RETURN IMPACT: the {resonance} keyhole is an impact keyhole, flown."
            );
        } else if converged {
            println!(
                "the {resonance} return misses by at least {:.0} km: the post-encounter orbit's \
                 spatial offset at the return, which no timing change removes \
                 (|ζ₂|/|ξ₂| = {timing:.2}, so the timing is spent).",
                r.distance_m / 1e3
            );
        } else {
            println!(
                "NOT A FLOOR: {:.0} km of return miss is still {:.1}x more TIMING than \
                 spatial (|ζ₂|/|ξ₂| = {timing:.2}), and Δv is a timing knob — so there is \
                 impulse left to spend and this number is not the orbits' offset.",
                r.distance_m / 1e3,
                timing
            );
        }
    }
}

/// One flown shot, with the return read in **its own** Öpik frame — the split
/// that says whether the residual is spatial (converged) or timing (not).
fn report(label: &str, shot: &KeyholeShot, nominal: &asteroid_core::BPlaneEncounter) {
    print!(
        "\n{label}: Δv {:.10} m/s → encounter-1 b {:.0} km ({:.2} R⊕ perigee), closed-form \
         a' {:.6} AU",
        shot.dv_m_s,
        shot.encounter.impact_parameter / 1e3,
        shot.encounter.perigee / nominal.earth_radius,
        shot.a_prime_m / AU_M
    );
    match shot.flown_return.as_ref() {
        None => println!("\n  no return inside the census gate"),
        Some(r) => {
            println!(
                "\n  return {:.0} km at {} ({:.3} yr later, {} approaches in gate)",
                r.distance_m / 1e3,
                r.epoch.as_hifitime(),
                r.years_after_first,
                r.approaches_in_gate
            );
            match (r.xi_m, r.zeta_m) {
                (Some(xi), Some(zeta)) => println!(
                    "  in the RETURN's own Öpik frame: ξ₂ {:.3} km (spatial offset), \
                     ζ₂ {:.3} km (timing), |ζ₂|/|ξ₂| = {:.3}",
                    xi / 1e3,
                    zeta / 1e3,
                    r.timing_share().unwrap_or(f64::NAN)
                ),
                _ => println!("  the return did not reduce to a b-plane (not hyperbolic?)"),
            }
        }
    }
}
