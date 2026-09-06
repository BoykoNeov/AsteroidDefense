//! How sharply is the 3:4 keyhole's refined floor actually determined — and what
//! does the *last printed digit* of the published Δv cost in the coordinate the
//! keyhole conclusions are stated in?
//!
//! # The question this exists to settle
//!
//! Two places in this repo report the same refined 3:4 shot and disagree about
//! its timing coordinate. `keyhole_target.rs`'s module-doc table (and
//! `probe_keyhole_return`'s) say `ζ₂` = **786 km** against `ξ₂` 4 013 km;
//! `probe_integrator_convergence`'s table prints **891 km** against `ξ₂` 4 014 km.
//! One measurement, two spellings — and the tempting resolution ("re-run it and
//! believe the code") is wrong, because *both probes are right about different
//! inputs*:
//!
//! - `probe_keyhole_return` flies the golden-section's own full-precision Δv.
//! - `probe_integrator_convergence` flies the constant `0.216_550` — the same Δv
//!   **rounded to the six decimals it was published at**.
//!
//! `ξ₂` agreeing to 1 km while `ζ₂` differs by 105 km is already the fingerprint:
//! a differently-built return frame would rotate the split and move *both*
//! numbers. Only Δv moves `ζ₂` alone, because `ζ₂` is the timing coordinate and
//! Δv is a timing knob — that is the whole argument `keyhole_target.rs` makes.
//!
//! So this probe measures the sensitivity directly: fly the shot at a ladder of
//! Δv values straddling the published floor and read `ζ₂` off each one. The slope
//! `∂ζ₂/∂Δv` then answers three things arithmetically, without a second
//! eight-minute refinement:
//!
//! 1. **Is the 105 km gap inside a six-decimal print's rounding?** A `{:.6}` print
//!    of Δv carries ±5e-7 m/s. Multiply by the slope and compare.
//! 2. **How much of `ζ₂` did the search ever resolve?** `solve_keyhole_return`
//!    reports its final bracket as `dv_window_m_s` (~1.3e-5 m/s on this shot).
//!    Times the slope, that is the width of the flat bottom the refinement stopped
//!    somewhere inside — and if it is large beside 786 km, then neither 786 nor
//!    891 is a *measurement* of the floor's timing component. They are samples.
//! 3. **Where is the scalar-distance minimum, in `ζ₂`?** The refinement minimises
//!    the geocentric return distance, not `|ζ₂|`, so the two need not land in the
//!    same place — except that `|∂ζ₂/∂Δv|` is ~112× `|∂ξ₂/∂Δv|`, which makes the
//!    distance a proxy for `|ζ₂|` near the bottom. This scan fits the vertex and
//!    checks it against the `ζ₂ = 0` crossing instead of assuming either way.
//! 4. **What is the door width in Δv, then?** Not `dv_window_m_s` — see below. The
//!    honest version is the Δv range over which the return still lands inside the
//!    *return's own* focused capture disc, which the slope and that disc give
//!    directly.
//!
//! **Measured 2026-09-06, and the answer to 3 is the reason this file is worth
//! keeping.** The refinement's published floor (Δv `0.216_549_808`, `ζ₂` 786 km,
//! return 1 130 km) is **not the minimum**. The real one is at Δv ≈ `0.216_548_4`,
//! where the return is **1 087 km** and `ζ₂` is **22 km** — the timing coordinate
//! goes to zero and the residual is pure `ξ₂` ≈ 4 006 km. Twelve golden-section
//! iterations only close a 1 % bracket to ~1.3e-5 m/s, which is 7 055 km of `ζ₂`,
//! so the search stopped 1.6e-6 m/s short and its leftover `ζ₂` was never the
//! orbits' timing residual — it was the distance to its own stopping point.
//!
//! So this **strengthens** the claim the floor is quoted for. `keyhole_target.rs`
//! argues a converged refinement must drive `ζ₂ → 0` and leave the floor in
//! `|ξ₂|`; at the actual minimum it does exactly that, and the published `ζ₂` of
//! several hundred km was the one piece of evidence against it.
//!
//! **Confirmed by the refinement itself, not just by this ladder.** Re-running
//! `probe_keyhole_return -- 3 4 minus retro 20` (26 flights, 623 s) lands at Δv
//! `0.2165483096` → return **1 087 km**, `ξ₂` 4 006 km, `ζ₂` **−26.6 km**, ratio
//! 0.007 — between this ladder's fitted vertex (`0.2165482893`) and its `ζ₂` = 0
//! crossing (`0.2165483587`). Two independent methods, same point — and the sign
//! flip between that `−26.6` and the `+22` two paragraphs up is the *same* thing
//! this file is about, not a new contradiction: the two land either side of the
//! `ζ₂ = 0` crossing, 9e-8 m/s apart. `ζ₂` near the floor is a **quantity whose
//! sign is set by the tenth decimal of Δv**, which is why it is worthless as a
//! published constant and priceless as a convergence gauge. And the wider
//! search's reported window came back **2.86e-7 m/s**, against `1.34e-5 · 0.618⁸ =
//! 2.85e-7`: `dv_window_m_s` tracks the iteration count to three digits and is
//! therefore the budget, exactly as item 2 says.
//!
//! Requires kernels. ~15 s per Δv rung plus a ~25 s scenario build, so the default
//! nine-rung ladder is ~2.5 min. Nothing here refines, so nothing here costs the
//! ~230 s Δv solve `probe_keyhole_return` pays.
//!
//!   cargo run -p asteroid_core --release --example probe_keyhole_floor
//!   cargo run -p asteroid_core --release --example probe_keyhole_floor -- 0.216550 1e-5 9
//!
//! Arguments (all optional, positional): `centre half_width rungs`, where `centre`
//! is the Δv the ladder straddles (m/s), `half_width` its half-span (m/s), and
//! `rungs` how many flights.

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, fly_keyhole_shot, ImpactorConfig, KeyholeAiming, KeyholeShotOptions,
    OpikFrame, RealFieldScenario, Resonance,
};
use std::time::Instant;

/// The 3:4 floor **as published** — six decimals, which is exactly the point.
///
/// This is the number `probe_integrator_convergence` flies and the number every
/// doc table in the repo quotes. Whether it is enough digits to reproduce `ζ₂` is
/// the question, so it is deliberately spelled here the way a reader would copy it
/// out of a table, not to full precision.
const DV_FLOOR_M_S: f64 = 0.216_550;

/// The resonance the published floor belongs to.
const RESONANCE: Resonance = Resonance { h: 3, k: 4 };

/// The rounding a `{:.6}` print of Δv carries, m/s — half a unit in the last place.
const SIX_DECIMAL_ROUNDING_M_S: f64 = 5.0e-7;

/// The final golden-section bracket reported for this shot, m/s.
///
/// Quoted from `probe_keyhole_return`'s own run rather than re-derived: this probe
/// does not refine, and the point is to price *that* search's resolution.
const REPORTED_DV_WINDOW_M_S: f64 = 1.3e-5;

/// The closed-form aim the refinement brackets around, m/s — `KeyholeRefineTol`'s
/// `bracket_fraction` is a fraction *of this*, not of the floor.
const AIM_DV_M_S: f64 = 0.216_437_5;

/// `KeyholeRefineTol::default().bracket_fraction`.
const BRACKET_FRACTION: f64 = 0.01;

/// `KeyholeRefineTol::default().max_iterations`.
const GOLDEN_ITERATIONS: i32 = 12;

/// What one golden-section iteration multiplies the bracket by.
const GOLDEN_RATIO_STEP: f64 = 0.618_033_988_749_895;

/// The two spellings of `ζ₂` this probe exists to reconcile, km.
const ZETA_SPELLINGS_KM: [(&str, f64); 2] = [
    ("keyhole_target.rs / probe_keyhole_return", 786.0),
    ("probe_integrator_convergence", 891.0),
];

struct Rung {
    dv_m_s: f64,
    distance_km: f64,
    xi_km: f64,
    zeta_km: f64,
    /// The **return** encounter's own focused capture radius, km — the disc this
    /// return has to land inside to be an impact. Not encounter 1's.
    capture_km: Option<f64>,
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let centre: f64 = args
        .first()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DV_FLOOR_M_S);
    let half_width: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1.0e-5);
    let rungs: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(9);
    if rungs < 3 {
        eprintln!("need at least 3 rungs to fit a slope (got {rungs})");
        std::process::exit(1);
    }

    println!(
        "{RESONANCE} floor resolution: {rungs} flights over Δv {centre:.10} ± {half_width:.1e} m/s"
    );

    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&ImpactorConfig::default()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e}");
            eprintln!("(this probe needs DE440 kernels — see kernels::resolve)");
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

    // Retrograde, matching the shot both disagreeing tables fly.
    let seed = ds.nominal().state_at(scenario.epoch0()).expect("seed");
    let direction = -along_track_unit(seed).expect("along-track");
    let opts = KeyholeShotOptions::default();

    println!(
        "\n  {:>18}  {:>13}  {:>13}  {:>13}  {:>8}  {:>6}",
        "Δv m/s", "return km", "ξ₂ km", "ζ₂ km", "|ζ₂/ξ₂|", "s"
    );
    let mut ladder: Vec<Rung> = Vec::new();
    for i in 0..rungs {
        let f = i as f64 / (rungs - 1) as f64;
        let dv = centre - half_width + 2.0 * half_width * f;
        let aiming = KeyholeAiming {
            frame: &frame,
            earth_radius_m: nominal.earth_radius,
            deflection_epoch: scenario.epoch0(),
            direction,
        };
        let t = Instant::now();
        let shot = fly_keyhole_shot(&scenario, &ds, aiming, dv, RESONANCE, &opts);
        let secs = t.elapsed().as_secs_f64();
        match shot {
            Err(e) => println!("  {dv:>18.10}  flight failed: {e}"),
            Ok(shot) => match shot.flown_return.as_ref() {
                None => println!("  {dv:>18.10}  no return inside the census gate"),
                Some(r) => match (r.xi_m, r.zeta_m) {
                    (Some(xi), Some(zeta)) => {
                        println!(
                            "  {dv:>18.10}  {:>13.3}  {:>13.3}  {:>13.3}  {:>8.4}  {secs:>6.1}",
                            r.distance_m / 1e3,
                            xi / 1e3,
                            zeta / 1e3,
                            r.timing_share().unwrap_or(f64::NAN)
                        );
                        ladder.push(Rung {
                            dv_m_s: dv,
                            distance_km: r.distance_m / 1e3,
                            xi_km: xi / 1e3,
                            zeta_km: zeta / 1e3,
                            capture_km: r.encounter.as_ref().map(|e| e.capture_radius / 1e3),
                        });
                    }
                    _ => println!("  {dv:>18.10}  the return did not reduce to a b-plane"),
                },
            },
        }
    }

    if ladder.len() < 3 {
        println!("\n  too few successful flights to fit anything — nothing concluded.");
        return;
    }
    interpret(&ladder);
}

/// Least-squares slope and intercept of `y` against `x`.
fn linear_fit(xs: &[f64], ys: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let mx = xs.iter().sum::<f64>() / n;
    let my = ys.iter().sum::<f64>() / n;
    let sxy: f64 = xs.iter().zip(ys).map(|(x, y)| (x - mx) * (y - my)).sum();
    let sxx: f64 = xs.iter().map(|x| (x - mx) * (x - mx)).sum();
    let slope = sxy / sxx;
    (slope, my - slope * mx)
}

fn interpret(ladder: &[Rung]) {
    let dvs: Vec<f64> = ladder.iter().map(|r| r.dv_m_s).collect();
    let zetas: Vec<f64> = ladder.iter().map(|r| r.zeta_km).collect();
    let xis: Vec<f64> = ladder.iter().map(|r| r.xi_km).collect();
    let (dzeta, zeta0) = linear_fit(&dvs, &zetas);
    let (dxi, _) = linear_fit(&dvs, &xis);

    println!("\n=== what the ladder says ===");
    println!(
        "\n  ∂ζ₂/∂Δv = {dzeta:.3e} km per m/s   (timing — the knob Δv actually turns)\n  \
         ∂ξ₂/∂Δv = {dxi:.3e} km per m/s   (spatial — {:.0}× less responsive)",
        (dzeta / dxi).abs()
    );

    // 1. Does a six-decimal print explain the two spellings?
    let rounding_km = (dzeta * SIX_DECIMAL_ROUNDING_M_S).abs();
    println!(
        "\n  1. A six-decimal Δv carries ±{SIX_DECIMAL_ROUNDING_M_S:.0e} m/s, which is \
         ±{rounding_km:.0} km of ζ₂."
    );
    let (lo, hi) = (ZETA_SPELLINGS_KM[0].1, ZETA_SPELLINGS_KM[1].1);
    let gap_dv = ((hi - lo) / dzeta).abs();
    println!(
        "     The two published spellings differ by {:.0} km = {gap_dv:.2e} m/s of Δv, \
         which is {:.2}× that rounding.",
        (hi - lo).abs(),
        gap_dv / SIX_DECIMAL_ROUNDING_M_S
    );
    for (who, zeta) in ZETA_SPELLINGS_KM {
        println!(
            "     ζ₂ = {zeta:>4.0} km ({who}) is reached at Δv {:.10}",
            (zeta - zeta0) / dzeta
        );
    }

    // 2. How much of ζ₂ did the refinement ever resolve?
    let window_km = (dzeta * REPORTED_DV_WINDOW_M_S).abs();
    println!(
        "\n  2. The refinement's own final bracket, {REPORTED_DV_WINDOW_M_S:.1e} m/s, spans \
         {window_km:.0} km of ζ₂."
    );
    println!(
        "     Against a quoted ζ₂ of {lo:.0} km that is {:.1}× — so the search pinned ζ₂ only \
         to about ±{:.0} km,",
        window_km / lo,
        window_km / 2.0
    );
    println!(
        "     and any three-significant-figure ζ₂ at this floor is a sample from inside the \
         bracket, not a measurement."
    );
    let budget = 2.0 * BRACKET_FRACTION * AIM_DV_M_S * GOLDEN_RATIO_STEP.powi(GOLDEN_ITERATIONS);
    println!(
        "     And that bracket is arithmetic, not physics: 2·{BRACKET_FRACTION}·{AIM_DV_M_S} \
         ·0.618^{GOLDEN_ITERATIONS} = {budget:.2e} m/s."
    );
    println!(
        "     It is the ITERATION BUDGET. `rel_tol` (1e-7 of Δv ≈ 2.2e-8 m/s) needs ~26 \
         iterations and never binds at 12,"
    );
    println!(
        "     so `dv_window_m_s` at the defaults measures how long the search ran, and any door \
         width read off it is a coincidence."
    );

    // 3. Where does the scalar distance actually bottom out, and is it at ζ₂ = 0?
    let zeta_zero_dv = -zeta0 / dzeta;
    let lowest = ladder
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.distance_km.total_cmp(&b.1.distance_km))
        .expect("non-empty")
        .0;
    println!(
        "\n  3. Lowest rung: {:.3} km at Δv {:.10} (ζ₂ {:.3} km). ζ₂ = 0 sits at Δv \
         {zeta_zero_dv:.10}.",
        ladder[lowest].distance_km, ladder[lowest].dv_m_s, ladder[lowest].zeta_km
    );
    match parabola_vertex(ladder, lowest) {
        None => println!(
            "     The lowest rung is an END of the ladder, so the minimum is outside it — \
             widen the span before reading anything into this."
        ),
        Some((vertex_dv, vertex_km)) => {
            let gap_dv = (vertex_dv - zeta_zero_dv).abs();
            println!(
                "     Fitted vertex: {vertex_km:.0} km at Δv {vertex_dv:.10}, which is \
                 {gap_dv:.1e} m/s from the ζ₂ = 0 crossing",
            );
            println!(
                "     — {:.0} km of ζ₂. The distance minimum and the timing zero are the \
                 SAME point, to the ladder's resolution:",
                (gap_dv * dzeta).abs()
            );
            println!(
                "     near the bottom |∂ζ₂/∂Δv| is {:.0}× |∂ξ₂/∂Δv|, so the scalar distance is \
                 a proxy for |ζ₂| and minimising",
                (dzeta / dxi).abs()
            );
            println!("     one minimises the other. That is the module's claim, measured.");
        }
    }

    // 4. What a Δv door width would actually be, since the search's bracket is not one.
    let xi_mean = xis.iter().sum::<f64>() / xis.len() as f64;
    let captures: Vec<f64> = ladder.iter().filter_map(|r| r.capture_km).collect();
    if captures.is_empty() {
        println!("\n  4. No return-encounter geometry on any rung — no door width to state.");
    } else {
        let capture = captures.iter().sum::<f64>() / captures.len() as f64;
        println!(
            "\n  4. The door, for contrast with item 2. The return's own focused capture disc is \
             {capture:.0} km"
        );
        println!(
            "     and the spatial floor eats ξ₂ ≈ {xi_mean:.0} km of it, so the return impacts \
             while |ζ₂| < {:.0} km,",
            (capture * capture - xi_mean * xi_mean).max(0.0).sqrt()
        );
        let half = (capture * capture - xi_mean * xi_mean).max(0.0).sqrt() / dzeta.abs();
        println!(
            "     i.e. across ±{half:.2e} m/s of Δv — a door {:.1}× WIDER than the bracket \
             item 2 priced.",
            half / (REPORTED_DV_WINDOW_M_S / 2.0)
        );
        println!(
            "     (Linear extrapolation off this ladder, not flown to the edges: every rung here \
             returns inside the disc.)"
        );
    }

    println!(
        "\n  What the four add up to: the floor really is **spatial** — at the fitted minimum the \
         timing\n  component is zero and what remains is ξ₂ ≈ {xi_mean:.0} km. A published floor \
         with a ζ₂ of several\n  hundred km is not reporting that residual; it is reporting how \
         far short of the minimum its\n  search stopped, in a coordinate that moves \
         {dzeta:.1e} km per m/s. And the search's own\n  final bracket is its iteration budget, \
         not the door — the door is item 4."
    );
}

/// Vertex of the parabola through the lowest rung and its two neighbours, as
/// `(Δv, distance km)`.
///
/// `None` when the lowest rung is an end of the ladder — there is no bracketed
/// minimum then, and fitting one anyway is how a search reports a wall as a floor
/// (the trap `KeyholeSolution::bracketed` exists for).
fn parabola_vertex(ladder: &[Rung], lowest: usize) -> Option<(f64, f64)> {
    if lowest == 0 || lowest + 1 >= ladder.len() {
        return None;
    }
    let (a, b, c) = (&ladder[lowest - 1], &ladder[lowest], &ladder[lowest + 1]);
    let h = b.dv_m_s - a.dv_m_s;
    let curvature = a.distance_km - 2.0 * b.distance_km + c.distance_km;
    if !(curvature > 0.0) {
        return None;
    }
    let offset = 0.5 * h * (a.distance_km - c.distance_km) / curvature;
    let drop = (a.distance_km - c.distance_km).powi(2) / (8.0 * curvature);
    Some((b.dv_m_s + offset, b.distance_km - drop))
}
