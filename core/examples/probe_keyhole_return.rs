//! Fly the 3:4 keyhole with the propagator: aim the shipping rock at the
//! resonant circle, let the real DE440 field carry it through the flyby and
//! four more revolutions, and measure how close it comes back.
//!
//! Everything in `keyhole.rs` is closed-form, and the module says so: absolute
//! placement carries the `r ≈ R⊕ₒᵣᵦ` error (~1.3e-4 in `a'`, ~12 h of timing over a
//! 3-year return, ~1.3e6 km of Earth motion). So the closed form can only *aim*;
//! whether a return actually happens, and how close it comes, has to be flown.
//! This probe does that, in three steps:
//!
//!   1. **Aim.** Solve the retrograde along-track Δv at the campaign start that
//!      puts the deflected b-point on the 3:4 circle (at the ξ the nudge
//!      naturally lands on). `required_dv` targets a perigee, so the circle point
//!      is converted to one through `b² = r_p² + 2μr_p/v∞²`. ~190 s.
//!   2. **Fly through.** Continue the deflected trajectory past the encounter
//!      for 3.6 years in the same field and census Earth close approaches with
//!      a wide gate (0.05 AU), because the closed form's timing error is far
//!      outside the shipping 500 000 km gate.
//!   3. **Refine.** The return miss is V-shaped in Δv (it is Earth's motion over
//!      the timing slip), so a golden-section search on Δv drives it to its
//!      floor. That floor is the physics: the post-encounter orbit's MOID-like
//!      offset at the return, which no timing change can remove. If the floor is
//!      inside the capture disc, the keyhole is an **impact** keyhole and this is
//!      the first resonant-return impact this project has flown; if not, it is a
//!      return that misses by the floor, which is just as much a measurement.
//!
//! **Measured (2026-09-02, shipping scenario):** the closed-form aim gave
//! Δv = 0.216438 m/s retrograde and a return at 53 841 km — about five capture
//! radii, better than the module doc's ~100 bound; three golden-section steps
//! brought it inside the disc and the floor is **0.216550 m/s → 1 130 km from
//! Earth's centre on 2042-12-31**, 3.00 yr after the 2040-01-01 flyby. Inside
//! Earth. The Δv window at the floor is ~1.3e-5 m/s, which at ~1e6 km of b per
//! m/s is ~13 km of b-plane — the same order as the closed form's 24.9 km far-end
//! keyhole width. `keyhole.rs` re-flies that Δv as a 15 s regression test.
//!
//! Requires kernels. ~4 min (the Δv solve ~230 s, then ~15 re-flies at 14 s).
//!
//!   cargo run -p asteroid_core --release --example probe_keyhole_return

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, closest_approach, find_close_approaches, DvSolveTol, EphemerisPerturber,
    ImpactorConfig, OpikFrame, RealFieldScenario, Resonance, ScanOptions, AU_M,
};
use nalgebra::Vector2;
use std::time::Instant;

const SHIPPING_SCAN_GATE_M: f64 = 5.0e8;
/// The return census gate: the closed form's ~1.3e6 km timing error, with room.
const RETURN_GATE_M: f64 = 0.05 * AU_M;
/// How long past the first encounter to fly: the 3-year return plus margin.
const RETURN_SPAN_YEARS: f64 = 3.6;
/// Continue from this far past the first encounter (well outside Earth's SOI).
const HANDOFF_DAYS_AFTER_CA: f64 = 30.0;
const RESONANCE: Resonance = Resonance { h: 3, k: 4 };

fn main() {
    let cfg = ImpactorConfig::default();
    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&cfg) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e}");
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
    let nominal = scenario.nominal_hit(&ds).expect("nominal hit");
    let t_ca = ds
        .nominal_encounter_epoch()
        .expect("epoch")
        .expect("an encounter");
    let (r_km, v_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
        .expect("Earth state");
    let frame = OpikFrame::new(&nominal, r_km * 1e3, v_km * 1e3, mu_sun).expect("frame");
    let circle = frame.resonant_circle(RESONANCE).expect("3:4 in reach");
    let epoch0 = scenario.epoch0();
    let seed = ds.nominal().state_at(epoch0).expect("seed");
    let prograde = along_track_unit(seed).expect("along-track");
    let retro = -prograde;

    // ---- step 1: aim -------------------------------------------------------
    //
    // A retrograde nudge lands at ξ ≈ +6 200 km (the map probe measured 6 166 at
    // 0.2 m/s); the circle's far branch at that ξ is the target. Held at the
    // nominal's own ξ, which the nudge barely moves.
    let xi = frame.project(&nominal.b_vector).x;
    let zeta = circle.center_zeta - (circle.radius * circle.radius - xi * xi).sqrt();
    let target = Vector2::new(xi, zeta);
    let b_target = target.norm();
    let c = frame.c();
    let perigee_target = -c + (c * c + b_target * b_target).sqrt();
    println!(
        "3:4 target: ξ {:.0} ζ {:.0} km → b {:.0} km, perigee {:.0} km ({:.2} R⊕); \
         closed-form a' there {:.6} AU (resonant {:.6})",
        xi / 1e3,
        zeta / 1e3,
        b_target / 1e3,
        perigee_target / 1e3,
        perigee_target / nominal.earth_radius,
        frame.post_encounter_semi_major_axis(target) / AU_M,
        circle.a_prime / AU_M
    );
    let t = Instant::now();
    let dv0 = ds
        .required_dv(
            epoch0,
            retro,
            perigee_target,
            DvSolveTol {
                rel_tol: 1.0e-3,
                ..Default::default()
            },
        )
        .expect("required_dv");
    println!(
        "retrograde Δv for that perigee: {dv0:.6} m/s   [{:.0} s]",
        t.elapsed().as_secs_f64()
    );

    // ---- steps 2 & 3: fly through and refine ----------------------------------
    //
    // One evaluation: re-fly the deflection, reduce encounter 1, hand off 30 d
    // past it, fly 3.6 yr more, census Earth approaches inside the wide gate.
    let fly = |dv: f64| -> Option<(f64, f64, f64, String)> {
        let (clock, _) = ds.deflected_trajectory(epoch0, dv * retro).ok()?;
        let ca1 = closest_approach(&clock, &earth, scan).ok()??;
        let enc1 = ca1.b_plane(nominal.mu, nominal.earth_radius).ok()?;
        let p1 = frame.project(&enc1.b_vector);
        let a_closed = frame.post_encounter_semi_major_axis(p1);
        let t_hand = ca1
            .epoch
            .shifted_by_seconds(HANDOFF_DAYS_AFTER_CA * 86_400.0);
        let hand = clock.state_at(t_hand).ok()?;
        let n = (RETURN_SPAN_YEARS * 365.25).ceil() as u32;
        let onward = scenario.propagate_free(t_hand, hand, 86_400.0, n).ok()?;
        let returns = find_close_approaches(
            &onward,
            &earth,
            ScanOptions {
                max_sample_dt: 6.0 * 3600.0,
                time_tol_seconds: 1.0e-3,
                max_distance: Some(RETURN_GATE_M),
            },
        )
        .ok()?;
        let best = returns
            .iter()
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())?;
        Some((
            best.distance,
            enc1.impact_parameter,
            a_closed,
            format!(
                "{} ({} approaches in gate)",
                best.epoch.as_hifitime(),
                returns.len()
            ),
        ))
    };

    let report = |dv: f64, r: &Option<(f64, f64, f64, String)>| match r {
        Some((d, b1, a, when)) => println!(
            "  Δv {dv:.6} m/s → encounter-1 b {:.0} km, a' {:.6} AU; return {:.0} km at {when}",
            b1 / 1e3,
            a / AU_M,
            d / 1e3
        ),
        None => println!(
            "  Δv {dv:.6} m/s → no return inside {:.3} AU",
            RETURN_GATE_M / AU_M
        ),
    };

    let t = Instant::now();
    let r0 = fly(dv0);
    report(dv0, &r0);
    println!("  [{:.0} s per evaluation]", t.elapsed().as_secs_f64());

    // Bracket: the return miss is ~linear in |Δv − Δv*|. Step ±1 % and widen
    // until the centre is the lowest of three.
    let mut lo = dv0 * 0.99;
    let mut hi = dv0 * 1.01;
    let miss = |dv: f64| fly(dv).map(|r| r.0).unwrap_or(f64::INFINITY);
    let (mut f_lo, mut f_hi) = (miss(lo), miss(hi));
    let mut f_mid = r0.as_ref().map(|r| r.0).unwrap_or(f64::INFINITY);
    let mut mid = dv0;
    println!(
        "bracket: {:.0} / {:.0} / {:.0} km",
        f_lo / 1e3,
        f_mid / 1e3,
        f_hi / 1e3
    );
    for _ in 0..6 {
        if f_mid <= f_lo && f_mid <= f_hi {
            break;
        }
        if f_lo < f_mid {
            hi = mid;
            f_hi = f_mid;
            mid = lo;
            f_mid = f_lo;
            lo = mid - (hi - mid);
            f_lo = miss(lo);
        } else {
            lo = mid;
            f_lo = f_mid;
            mid = hi;
            f_mid = f_hi;
            hi = mid + (mid - lo);
            f_hi = miss(hi);
        }
        println!(
            "  widen: Δv {lo:.6}/{mid:.6}/{hi:.6} → {:.0} / {:.0} / {:.0} km",
            f_lo / 1e3,
            f_mid / 1e3,
            f_hi / 1e3
        );
    }

    // Golden-section on [lo, hi].
    let phi = 0.5 * (5.0f64.sqrt() - 1.0);
    let mut a = lo;
    let mut b = hi;
    let mut x1 = b - phi * (b - a);
    let mut x2 = a + phi * (b - a);
    let (mut f1, mut f2) = (miss(x1), miss(x2));
    for i in 0..12 {
        if f1 < f2 {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - phi * (b - a);
            f1 = miss(x1);
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + phi * (b - a);
            f2 = miss(x2);
        }
        println!(
            "  golden {i}: Δv [{a:.7}, {b:.7}] m/s, return {:.1} / {:.1} km",
            f1 / 1e3,
            f2 / 1e3
        );
        if (b - a) < 1.0e-7 * dv0 {
            break;
        }
    }
    let dv_best = if f1 < f2 { x1 } else { x2 };
    let best = fly(dv_best);
    println!("\nfloor:");
    report(dv_best, &best);
    if let Some((d, _, _, _)) = best {
        println!(
            "  return miss {:.0} km = {:.2} capture radii = {:.3} LD; keyhole Δv window \
             ~{:.2e} m/s of {:.4}",
            d / 1e3,
            d / nominal.capture_radius,
            d / 384_400e3,
            (b - a),
            dv_best
        );
        if d <= nominal.capture_radius {
            println!("  ** RESONANT-RETURN IMPACT: the 3:4 keyhole is an impact keyhole, flown.");
        } else {
            println!(
                "  the 3:4 return misses by at least {:.0} km: the post-encounter orbit's \
                 spatial offset at the return, which no timing change removes.",
                d / 1e3
            );
        }
    }
    println!("total {:.0} s", t.elapsed().as_secs_f64());
}
