//! Find a deflected geometry whose perigee sits **outside** Earth — the geometry
//! the `J2` follow-up needs.
//!
//! HANDOFF records that Earth's `J2` moves the shipping nominal's b-plane perigee
//! by 1.33 km, and that this number *grazes a validity boundary*: the `J2`
//! expansion holds only outside `R_eq`, and the shipping nominal is a designed
//! **impact** whose closest approach is 3000 km — well inside Earth. The honest
//! follow-up is to measure the same term on a geometry that stays outside the
//! body, and this probe picks that geometry on numbers.
//!
//! **Why a deflected pass and not a wider `b_offset_km`.** Measured first, and the
//! offset knob cannot do it: [`RealFieldScenario::build`] verifies its designed
//! impact round-trips, so any offset past the capture radius is rejected
//! (`perigee 1.500e7 m ≥ capture radius 7.711e6 m (not a hit)` at 15 000 km) rather
//! than built as a miss. That leaves the deflected pass — which is also the case
//! that actually matters, since every successful deflection is one.
//!
//! Nothing here is predicted: the impulse is *solved* for a target perigee
//! (`required_dv_along_track`) and then the geometry it actually reaches is
//! reported — perigee, `|B|`, capture radius, and whether the pass stayed inside
//! the scan gate at all (one that leaves it has no finite perigee to measure).
//!
//! Usage: set ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS and run:
//!   cargo run -p asteroid_core --release --example probe_miss_geometry

use asteroid_core::deflection::DvSolveTol;
use asteroid_core::geometry::EARTH_EQUATORIAL_RADIUS_M as R_E;
use asteroid_core::scenario::{ImpactorConfig, RealFieldScenario};

fn main() {
    if asteroid_core::kernels::resolve().is_none() {
        eprintln!(
            "no DE kernel pair resolved — set ASTEROID_DE_KERNEL/ASTEROID_PLANETARY_CONSTANTS"
        );
        std::process::exit(2);
    }

    let sc = RealFieldScenario::build(&ImpactorConfig::default()).expect("scenario builds");
    let ds = sc.deflection().expect("deflection");
    let nominal = ds
        .nominal_encounter()
        .expect("nominal reduces")
        .expect("nominal is a hit");
    println!(
        "nominal (the designed impact): perigee {:.1} km = {:.3} R_eq, |B| {:.1} km, \
         b_cap {:.1} km, v_inf {:.3} km/s",
        nominal.perigee / 1e3,
        nominal.perigee / R_E,
        nominal.impact_parameter / 1e3,
        nominal.capture_radius / 1e3,
        nominal.v_inf / 1e3,
    );

    // Deflect one year out, not at the campaign start. Measured first: every
    // `required_dv` bisection step re-propagates from the impulse to the span end,
    // so an impulse at t=0 makes each of ~30 steps a full 12 yr flight (the first
    // run of this probe was still going after 18 minutes). A one-year lead costs a
    // twelfth of that per step and answers the same question — what is being chosen
    // here is a *perigee*, and the lead time only sets how much Δv buys it.
    let t_d = sc
        .impact_epoch()
        .shifted_by_seconds(-1.0 * 365.25 * 86_400.0);
    println!(
        "\ndeflection 1.0 yr before impact, along-track\n\n{:>10}  {:>12}  {:>10}  \
         {:>12}  {:>12}  {:>10}  {:>8}",
        "target R_eq", "dv m/s", "perigee km", "peri/R_eq", "|B| km", "b_cap km", "verdict"
    );

    for target_re in [2.0_f64, 3.0, 4.0] {
        let target_m = target_re * R_E;
        let dv = match ds.required_dv_along_track(t_d, target_m, DvSolveTol::default()) {
            Ok(v) => v,
            Err(e) => {
                println!("{target_re:>10.1}  solve failed: {e}");
                continue;
            }
        };
        let dir = asteroid_core::deflection::along_track_unit(
            ds.nominal().state_at(t_d).expect("nominal state at t_d"),
        )
        .expect("along-track direction");
        match ds.evaluate(t_d, dv * dir) {
            Ok(Some(e)) => println!(
                "{:>10.1}  {:>12.6}  {:>10.1}  {:>12.3}  {:>12.1}  {:>10.1}  {:>8}",
                target_re,
                dv,
                e.perigee / 1e3,
                e.perigee / R_E,
                e.impact_parameter / 1e3,
                e.capture_radius / 1e3,
                if e.impact_parameter > e.capture_radius {
                    "MISS"
                } else {
                    "hit"
                },
            ),
            Ok(None) => println!(
                "{target_re:>10.1}  {dv:>12.6}  left the scan gate — no finite perigee to measure"
            ),
            Err(e) => println!("{target_re:>10.1}  {dv:>12.6}  evaluate failed: {e}"),
        }
    }
}
