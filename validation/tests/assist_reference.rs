//! Tier-1 trajectory check against **ASSIST**, the §6 trajectory oracle
//! (HANDOFF §6, §10.7 batch 2c) — rung 3 of the §6 oracle ladder.
//!
//! Rung 1 (`free_invariants.rs`) confirms the propagator conserves what a map
//! must; rung 2 (`kepler_reference.rs`) pins the *two-body* numbers against
//! hapsira. This rung validates the **full Tier-1 force model** — the asteroid as
//! a test particle in the DE440 ephemeris field — against ASSIST, which the same
//! DE-field test-particle configuration ASSIST itself validates to ~metre level
//! vs JPL. ASSIST *is* our declared config (§6), so it is the trajectory oracle
//! from Tier 1 onward, not REBOUND (which self-gravitates the planets and is a
//! different dynamical system).
//!
//! # The comparison is dynamics, not a units/frame/constant slip
//! The fixture (`pyref/generate_assist_fixture.py` →
//! `fixtures/assist_tier1.json`) integrates asteroid (3666) Holman with ASSIST
//! configured to **point-mass gravity only**, from the *same* 11 bodies this test
//! sums — Sun + 8 planets + Moon + **Pluto** — with GR / Sun&Earth harmonics /
//! the 16 asteroid perturbers / A1-A3 non-gravs all **off**. Everything the two
//! sides must share is pinned identically (see the fixture's `frame_note` /
//! `provenance`): the 11-body point-mass force model, the barycentric SSB ICRF
//! frame, SI units (AU pinned equal to ASSIST's own AU), and TDB-seconds-past-
//! J2000 epochs.
//!
//! # Why 11 bodies here when the shipping field carries 10
//! ASSIST's direct-gravity term (harmonics/GR off) sums eleven bodies including
//! **Pluto** (`src/forces.c`, `order[ASSIST_BODY_NPLANETS]`); the shipping
//! [`tier1_perturber_field`](asteroid_core::tier1_perturber_field) carries the ten
//! of §5's locked "Sun + 8 planets + Moon". To validate the *machinery* against
//! the oracle at a tight, GM-floor-limited tolerance — not one loose enough to
//! swallow Pluto's pull and thereby hide a real μ-slip or rotation bug — this
//! test adds Pluto (NAIF 9) so both sides integrate the identical system. The
//! cost of the shipping field's Pluto omission is quantified separately by
//! [`pluto_omission_effect_over_arc`].
//!
//! # The residual floor is the GM source (advisor note)
//! Both sides read the *same* DE440 positions (ANISE from `de440s.bsp`, ASSIST
//! from the `.440`), so positions agree to machine precision and the dop853
//! integrator is held tight (rtol/atol `1e-12`, well below the GM floor). The one
//! remaining physics-input difference is GM: ASSIST reads the DE440 header,
//! `asteroid_core` reads ANISE's `pck11.pca`. [`anise_gm_matches_de440`] measures
//! that per-body delta directly — a large one is a *finding*, not a tolerance to
//! loosen — and the residual here should *track* it.
//!
//! Gated on `ASTEROID_DE_KERNEL` (a DE `.bsp`, e.g. `de440s.bsp`) +
//! `ASTEROID_PLANETARY_CONSTANTS` (a `.pca`, e.g. `pck11.pca`); skips green when
//! either is unset so CI stays offline, mirroring the other kernel-gated tests.

use std::collections::BTreeMap;
use std::sync::Arc;

use anise::constants::frames::{
    EARTH_J2000, JUPITER_BARYCENTER_J2000, MARS_BARYCENTER_J2000, MERCURY_J2000, MOON_J2000,
    NEPTUNE_BARYCENTER_J2000, PLUTO_BARYCENTER_J2000, SATURN_BARYCENTER_J2000, SUN_J2000,
    URANUS_BARYCENTER_J2000, VENUS_J2000,
};
use anise::prelude::Frame;
use asteroid_core::ephemeris::{Ephemeris, KM3_S2_TO_M3_S2};
use asteroid_core::forces::point_mass::{Perturber, PointMassGravity};
use asteroid_core::perturber_field::EphemerisPerturber;
use asteroid_core::{tier1_perturber_field, Dop853, Epoch, Integrator, StateVector};
use nalgebra::Vector3;
use serde::Deserialize;

const FIXTURE: &str = include_str!("../fixtures/assist_tier1.json");

/// Position relative-error bound between the dop853 11-body track and ASSIST.
/// Grounded in the observed residual (printed below, run with `--nocapture`):
/// worst **4.5e-11** at 730 d, *growing monotonically with arc length* — the
/// GM-delta-driven secular drift the residual is expected to track (the integrator
/// at rtol/atol 1e-12 sits far beneath it). The bound is **~2.2× that floor**,
/// chosen deliberately so it is *sensitive to a single dropped small perturber*:
/// omitting Pluto costs ~55 m over the arc = ~1.2e-10 relative (see
/// [`pluto_omission_effect_over_arc`]), which **exceeds this bound** — so the
/// claim "a force term left on/off blows through it" is actually true here, not
/// just for the orders-of-magnitude-larger frame/rotation/major-μ errors. The
/// residual is deterministic given pinned ANISE 0.10.3 + fixed kernels + the
/// committed fixture (cross-platform libm variance is ~1e-13), so 2.2× is ample.
/// A bonus consequence of a floor this low: it rules out a **J2000-vs-ICRF frame
/// bias** — a ~20 mas rotation would surface at ~1e-7, not 4.5e-11, proving the
/// two sides' frames are identical. See [`anise_gm_matches_de440`] for the floor's
/// source; a residual >> it means a structural bug, not a tolerance to loosen.
const POS_REL_TOL: f64 = 1e-10;
/// Velocity bound. Observed worst **3.3e-11** (at 365 d); held at the same order
/// as position (~3× over the floor) — velocity residuals track the position ones
/// through the shared force field.
const VEL_REL_TOL: f64 = 1e-10;
/// Per-body ANISE(pck11) − DE440(header) GM relative-delta bound. pck11 is *not*
/// bit-identical to the DE440 header: the observed worst is **Mercury ~4.0e-6**
/// (pck11 carries a different Mercury GM than DE440), with Mars/Moon ~1.4e-8 and
/// Jupiter ~5.5e-9; the Sun agrees to ~5e-12. This bound sits just above Mercury
/// so the deltas are *documented and asserted stable*, not silently absorbed —
/// while a gross mapping error (wrong body → O(1) delta) still trips. Dynamically
/// the floor is Jupiter's (largest mass × its 5.5e-9), not Mercury's (μ-ratio
/// ~1.6e-7 makes its 4e-6 negligible); see [`tier1_field_matches_assist`].
const GM_REL_TOL: f64 = 1e-5;

#[derive(Deserialize)]
struct Fixture {
    bodies: Vec<String>,
    de440_gm_km3_s2: BTreeMap<String, f64>,
    epoch0_tdb_seconds_past_j2000: f64,
    initial_state: State,
    samples: Vec<Sample>,
}

#[derive(Deserialize)]
struct State {
    position_m: [f64; 3],
    velocity_m_s: [f64; 3],
}

#[derive(Deserialize)]
struct Sample {
    days: f64,
    dt_s: f64,
    position_m: [f64; 3],
    velocity_m_s: [f64; 3],
}

fn load() -> Fixture {
    serde_json::from_str(FIXTURE).expect("assist fixture parses")
}

/// Name → ANISE [`Frame`] for the 11 comparison bodies. The single mapping used
/// for *both* the point-mass field and the GM check, so the mass a perturber's μ
/// describes always matches the body its position tracks (the §5 pairing rule).
/// Earth is the geocenter (399) with the Moon (301) separate — never the EMB;
/// Mars…Neptune and Pluto are DE barycenters; Mercury/Venus are body centers.
fn frame_for_body(name: &str) -> Option<Frame> {
    Some(match name {
        "Sun" => SUN_J2000,
        "Mercury" => MERCURY_J2000,
        "Venus" => VENUS_J2000,
        "Earth" => EARTH_J2000,
        "Moon" => MOON_J2000,
        "Mars" => MARS_BARYCENTER_J2000,
        "Jupiter" => JUPITER_BARYCENTER_J2000,
        "Saturn" => SATURN_BARYCENTER_J2000,
        "Uranus" => URANUS_BARYCENTER_J2000,
        "Neptune" => NEPTUNE_BARYCENTER_J2000,
        "Pluto" => PLUTO_BARYCENTER_J2000,
        _ => return None,
    })
}

/// GM (SI m³/s²) for `body`/`frame`: the *shipping* source (ANISE pck11) where it
/// resolves, else the oracle's own DE440 value from the fixture. Only Pluto needs
/// the fallback — `pck11.pca` carries no BODY9_GM — and using ASSIST's own Pluto
/// GM for the exact-match body is correct, not a fudge (the 10 shipping bodies
/// still use the shipping source, which is what [`tier1_field_matches_assist`]
/// validates). The fallback is logged so the pck11 Pluto gap stays visible.
fn resolve_gm_m3_s2(eph: &Ephemeris, fixture: &Fixture, body: &str, frame: Frame) -> f64 {
    match eph.gm_km3_s2(frame) {
        Ok(km3_s2) => km3_s2 * KM3_S2_TO_M3_S2,
        Err(_) => {
            let de440 = *fixture.de440_gm_km3_s2.get(body).unwrap_or_else(|| {
                panic!("{body} GM absent from both ANISE (pck11) and the fixture")
            });
            eprintln!(
                "note: {body} GM absent from pck11 — using the oracle's own DE440 \
                 value {de440} km³/s² for the exact-match comparison body"
            );
            de440 * KM3_S2_TO_M3_S2
        }
    }
}

/// Build a point-mass field over the fixture's bodies: position from `eph` for
/// each body's frame, GM via [`resolve_gm_m3_s2`] (shipping pck11, DE440 fallback
/// for Pluto). Position and GM are driven from the *same* frame per body (the §5
/// pairing rule).
fn build_comparison_field(eph: &Arc<Ephemeris>, fixture: &Fixture) -> PointMassGravity {
    let perturbers = fixture
        .bodies
        .iter()
        .map(|body| {
            let frame = frame_for_body(body).unwrap_or_else(|| panic!("unknown body {body:?}"));
            let mu = resolve_gm_m3_s2(eph, fixture, body, frame);
            Perturber::new(mu, EphemerisPerturber::new(Arc::clone(eph), frame))
        })
        .collect();
    PointMassGravity::new(perturbers)
}

/// Load the ephemeris through the shared resolver (environment, else a
/// conventional directory), or `None` — skip, green — when no pair resolves.
/// Shared by the tests below. `ASTEROID_REQUIRE_KERNELS` turns that skip into a
/// hard failure, which is the only way a reader of a green log can tell these
/// ASSIST comparisons actually ran.
fn gated_ephemeris() -> Option<Arc<Ephemeris>> {
    let k = asteroid_core::kernels::resolve_for_test("the ASSIST reference comparison")?;
    let (bsp, pca) = k.as_strs();
    Some(Arc::new(
        Ephemeris::load(bsp)
            .expect("load DE kernel")
            .with_constants(pca)
            .expect("load planetary constants"),
    ))
}

/// Relative error `‖c − r‖ / ‖r‖`.
fn rel_err(computed: Vector3<f64>, reference: [f64; 3]) -> f64 {
    let r = Vector3::new(reference[0], reference[1], reference[2]);
    (computed - r).norm() / r.norm()
}

/// The headline check: dop853 in the 11-body DE field reproduces ASSIST's
/// Holman track over two years to the GM-floor tolerance.
#[test]
fn tier1_field_matches_assist() {
    let Some(eph) = gated_ephemeris() else { return };
    let fixture = load();

    // Comparison field = the fixture's 11 bodies (membership driven by the
    // fixture itself), positions + GM from ANISE, Pluto's μ from the oracle.
    assert_eq!(fixture.bodies.len(), 11, "expected the 11-body ASSIST comparison set");
    let field = build_comparison_field(&eph, &fixture);

    // rtol/atol 1e-12: the integrator floor must sit well below the GM floor, or
    // it — not the physics — would set the residual (advisor note). Cf. the
    // dop853_adaptive test's ~1.5e-11 worst-case at this tolerance.
    let integrator = Dop853::new().with_tolerances(1e-12, 1e-12);

    let epoch0 = Epoch::from_tdb_seconds_past_j2000(fixture.epoch0_tdb_seconds_past_j2000);
    let state0 = StateVector::from_components(
        fixture.initial_state.position_m[0],
        fixture.initial_state.position_m[1],
        fixture.initial_state.position_m[2],
        fixture.initial_state.velocity_m_s[0],
        fixture.initial_state.velocity_m_s[1],
        fixture.initial_state.velocity_m_s[2],
    );

    let mut worst_pos = 0.0_f64;
    let mut worst_vel = 0.0_f64;
    for s in &fixture.samples {
        // Each sample is an independent from-IC integration to that epoch — a
        // clean per-epoch oracle comparison (dop853 sub-steps adaptively within).
        // Note the dt=0 sample is near-vacuous here (it reads the IC from the
        // fixture and steps by 0, comparing those numbers to themselves) — a zero-
        // step identity check, not an oracle data point; the real evidence is the
        // dt>0 samples.
        let state = integrator
            .step(&field, epoch0, &state0, s.dt_s)
            .unwrap_or_else(|e| panic!("dop853 step to {} d failed: {e}", s.days));

        let pos_err = rel_err(state.position, s.position_m);
        let vel_err = rel_err(state.velocity, s.velocity_m_s);
        println!(
            "t={:>5.0} d  pos_rel {:.3e}  vel_rel {:.3e}",
            s.days, pos_err, vel_err
        );
        assert!(
            pos_err < POS_REL_TOL && vel_err < VEL_REL_TOL,
            "at {} d: pos_rel {:.3e} / vel_rel {:.3e} exceed tol ({:.0e} / {:.0e}) — \
             a residual >> the measured GM floor points to a structural bug \
             (frame, sign, a rotation, or a force term not actually off), not a \
             tolerance to loosen",
            s.days, pos_err, vel_err, POS_REL_TOL, VEL_REL_TOL
        );
        worst_pos = worst_pos.max(pos_err);
        worst_vel = worst_vel.max(vel_err);
    }
    println!("worst vs ASSIST over 730 d: pos_rel {worst_pos:.3e}, vel_rel {worst_vel:.3e}");
}

/// The residual floor's source: ANISE's `pck11.pca` GMs vs the DE440-header GMs
/// ASSIST integrates with (recorded in the fixture from `gm_de440.tpc`). Measures
/// the per-body relative delta directly, so the headline residual is interpretable
/// (it should track the largest delta) and a materially different GM source shows
/// up as a named finding rather than a mysteriously loose tolerance.
#[test]
fn anise_gm_matches_de440() {
    let Some(eph) = gated_ephemeris() else { return };
    let fixture = load();

    let mut worst = 0.0_f64;
    let mut worst_body = String::new();
    for (body, &de440_km3_s2) in &fixture.de440_gm_km3_s2 {
        let frame = frame_for_body(body).unwrap_or_else(|| panic!("unknown body {body:?}"));
        // Pluto: pck11 has no BODY9_GM, so the shipping source can't supply it —
        // itself a finding. Report and skip (it can't set a delta with itself).
        let Ok(anise_km3_s2) = eph.gm_km3_s2(frame) else {
            println!("{body:<8} absent from pck11 (no shipping GM) — DE440 {de440_km3_s2:.9e}");
            continue;
        };
        let rel = (anise_km3_s2 - de440_km3_s2).abs() / de440_km3_s2;
        println!(
            "{body:<8} ANISE {anise_km3_s2:.9e}  DE440 {de440_km3_s2:.9e}  rel {rel:.3e}"
        );
        if rel > worst {
            worst = rel;
            worst_body = body.clone();
        }
    }
    println!("worst resolvable GM delta: {worst:.3e} ({worst_body})");
    assert!(
        worst < GM_REL_TOL,
        "{worst_body} GM differs {worst:.3e} between ANISE pck11 and DE440 header — \
         the shipping GM source disagrees with the oracle's; this is the residual \
         floor of tier1_field_matches_assist and a finding to reconcile, not to \
         absorb"
    );
}

/// Quantify *and guard* the cost of the shipping field's Pluto omission (§5 locks
/// 10 bodies; ASSIST — our declared config — carries 11). Propagates Holman under
/// the 10-body [`tier1_perturber_field`] and the 11-body field over the same arc:
/// reports the max position divergence (~55 m over 2 yr — the number that informs
/// whether Pluto should join the shipping field; pure Rust 10-vs-11, no ASSIST),
/// and **asserts** it (a) is present in a sane band and (b) exceeds `POS_REL_TOL`
/// — the latter is what makes [`tier1_field_matches_assist`]'s Pluto inclusion a
/// genuine guard (drop Pluto there and it fails) rather than a prose claim.
#[test]
fn pluto_omission_effect_over_arc() {
    let Some(eph) = gated_ephemeris() else { return };
    let fixture = load();

    let ten = tier1_perturber_field(&eph).expect("build 10-body tier-1 field");
    // 11-body = the same fixture field the headline test uses (Pluto μ from the
    // oracle, since pck11 lacks it).
    let eleven = build_comparison_field(&eph, &fixture);

    let integrator = Dop853::new().with_tolerances(1e-12, 1e-12);
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(fixture.epoch0_tdb_seconds_past_j2000);
    let state0 = StateVector::from_components(
        fixture.initial_state.position_m[0],
        fixture.initial_state.position_m[1],
        fixture.initial_state.position_m[2],
        fixture.initial_state.velocity_m_s[0],
        fixture.initial_state.velocity_m_s[1],
        fixture.initial_state.velocity_m_s[2],
    );

    let mut worst_m = 0.0_f64;
    let mut worst_rel = 0.0_f64;
    for s in &fixture.samples {
        let s10 = integrator.step(&ten, epoch0, &state0, s.dt_s).expect("10-body step");
        let s11 = integrator.step(&eleven, epoch0, &state0, s.dt_s).expect("11-body step");
        let diff = (s10.position - s11.position).norm();
        let rel = diff / s11.position.norm();
        println!("t={:>5.0} d  |Δr(10 vs 11)| {:.3e} m  (rel {:.3e})", s.days, diff, rel);
        worst_m = worst_m.max(diff);
        worst_rel = worst_rel.max(rel);
    }
    println!("Pluto omission: max |Δr| over 730 d = {worst_m:.3e} m (rel {worst_rel:.3e})");

    // Guard, not just report: Pluto's pull must be present (non-zero) and land in
    // a physically-sane band — a silent zero (e.g. its μ dropped to 0) would make
    // this test and the headline field indistinguishable from 10-body.
    assert!(
        (10.0..=500.0).contains(&worst_m),
        "Pluto omission {worst_m:.3e} m outside the expected ~55 m band — its term \
         vanished or blew up"
    );
    // And it must *exceed* the headline position tolerance: that is precisely why
    // dropping Pluto from build_comparison_field would fail tier1_field_matches_assist
    // (i.e. the 11-body field's Pluto inclusion is genuinely guarded, not assumed).
    assert!(
        worst_rel > POS_REL_TOL,
        "Pluto's relative effect {worst_rel:.3e} is within POS_REL_TOL {POS_REL_TOL:.0e} — \
         the headline test would NOT catch a dropped Pluto; tighten POS_REL_TOL"
    );
}
