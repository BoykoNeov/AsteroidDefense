//! **Tier-2 capstone (HANDOFF §5/§6).** Does *our* field, integrating a real
//! near-Earth asteroid's own initial state, reproduce JPL's published trajectory
//! — and does switching on each Tier-2 term (1PN relativity, then Yarkovsky)
//! measurably move our trajectory *toward* JPL's own solution?
//!
//! This is the payoff the 1PN and Yarkovsky terms were built for. The isolation
//! oracles proved each term is internally right (Mercury precession in
//! [`forces::relativity`](asteroid_core::forces::relativity); the secular
//! `⟨da/dt⟩` in [`forces::yarkovsky`](asteroid_core::forces::yarkovsky)). Those
//! answer "is the term the physics it claims to be?" This answers a harder
//! question: "put on the real DE440 field and seeded from a real object's real
//! state, does turning the term on make us agree better with JPL?"
//!
//! # It is a residual *curve*, not a "match"
//!
//! Nothing here claims we reproduce Horizons. JPL's solution carries physics we
//! do not — every planet's relativity (we model the Sun's 1PN only), the main
//! belt (the sixteen sb441 bodies are scenery here, **not** enrolled as forces),
//! and a radial non-grav term `A1` we do not model at all. So there is a residual
//! **floor** set by what our field omits, and the residual against JPL is a
//! function of arc length. The result is the *shape* of that curve as terms
//! switch on, and the honest reading of where each term rises above the floor —
//! never an amplified "it matches."
//!
//! # The discriminator is a control run
//!
//! Same shape as the isolation oracles' control runs (term-off ≪ term-on). Here
//! the control is the trajectory itself: integrate the identical seed through
//!
//!   1. Tier-1 only  (Sun + 8 planets + Moon point masses),
//!   2. + 1PN relativity,
//!   3. + Yarkovsky at the object's *real* transverse `A2`,
//!
//! and measure each against JPL's own held-out states. "Term on reduces the
//! residual against truth" is the claim; the numbers below are what it measures.
//!
//! # The two hazards this test is built around
//!
//! **Frame.** The `.neo` tables are heliocentric ICRF; the integrator is SSB. Both
//! directions go through the *same* Sun barycentric state ([`sun_ssb`]): seed as
//! `r_ssb = r_helio + r_sun_ssb`, compare as `r_helio = r_ssb − r_sun_ssb`. Same
//! source both ways, so the Sun's ~10⁶ km barycentric wobble cancels out of the
//! residual instead of swamping it. Truth is always the **raw held-out sample**
//! ([`Neo::sample`]), never the Hermite interpolation — otherwise we would be
//! measuring our own self-consistency.
//!
//! **`A2` is the real object's, in the term's units.** Apophis' SBDB solution
//! publishes `A2 = −2.902e−14 au/d²` (σ = 1.86e−16 — the Yarkovsky signal is
//! well-determined), with the model constants (`R0 = 1 au`, exponent 2) that are
//! exactly [`YarkovskyA2::standard`]'s form. Converted once, in [`APOPHIS_A2_SI`].
//! We use Apophis — not Bennu (whose modern solution models SRP, not an `A2`) and
//! not Didymos (no non-grav model) — and a **pre-2029 arc**, because the 2029
//! Earth flyby is the worst case for both interpolation and integration.
//!
//! # What it measures (de440s, 8-year arc, this machine)
//!
//! ```text
//! yr   Tier-1(km)   +GR(km)   +GR+Yk(km)   GR factor   Yk helps by
//!  1       22.8        1.5         2.2         15x         −0.7   (Yk below floor)
//!  3      100.0        0.6         7.8        175x         −7.3
//!  5      297.2       12.9        17.2         23x         −4.2
//!  8      176.8       37.8        18.6          5x        +19.2   (Yk clears floor)
//! ```
//!
//! 1PN relativity is the headline: it cuts the residual against JPL by 5×–175× at
//! every epoch — the term is not merely self-consistent, it pulls a real NEO's
//! trajectory an order of magnitude closer to JPL's own relativistic solution.
//! Yarkovsky is a **secular** signal: below the model's floor for the first few
//! years (where it adds noise, not skill), it clears the floor as its along-track
//! drift grows, halving the residual by the end of the arc. That it only emerges
//! over a multi-year baseline is itself the measured result — and the thing that
//! motivates the next menu item, enrolling the sixteen perturbers to lower the
//! floor.

use std::sync::Arc;

use anise::constants::frames::SUN_J2000;
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::forces::relativity::Relativity1PN;
use asteroid_core::forces::yarkovsky::YarkovskyA2;
use asteroid_core::{
    tier1_perturber_field, Clock, CompositeForce, Dop853, EphemerisPerturber, Epoch, StateVector,
};

/// One AU in metres.
const AU_M: f64 = 1.495_978_707e11;
/// Seconds per day (the `.neo` cadence, and the au/d² denominator).
const DAY_S: f64 = 86_400.0;

/// Apophis' JPL transverse non-grav parameter in the term's units (m/s² at 1 AU).
/// SBDB: `A2 = −2.902e−14 au/d²`; au/d² → m/s² is `AU_M / DAY_S²`. Fetched from
/// the JPL Small-Body Database, not recalled — the value a solution is *built
/// with* is the value it must be compared against.
const APOPHIS_A2_SI: f64 = -2.902e-14 * AU_M / (DAY_S * DAY_S);

/// Seed index into `apophis.neo` (~2020-01-31) and yearly held-out checkpoints,
/// the last (~2028) staying short of the ~sample-3390 flyby closest approach.
const SEED_INDEX: usize = 30;
const CHECK_YEARS: usize = 8;
const SAMPLES_PER_YEAR: usize = 365;

#[test]
fn apophis_own_integration_converges_to_horizons_as_tier2_terms_switch_on() {
    let Some(k) = asteroid_core::kernels::resolve_for_test(
        "Tier-2 capstone: Apophis integration vs Horizons",
    ) else {
        return;
    };
    let (bsp, pca) = k.as_strs();
    let eph = Ephemeris::load(bsp)
        .and_then(|e| e.with_constants(pca))
        .expect("load DE pair");
    let eph = Arc::new(eph);
    let mu_sun = eph.gm_km3_s2(SUN_J2000).expect("sun gm") * 1e9;

    let bodies =
        asteroid_core::horizons::load_all_for_test("Tier-2 capstone (needs apophis.neo)");
    let Some(apophis) = bodies.iter().find(|n| n.designation() == "99942") else {
        // Kernels/tables present but this particular object is not — a legitimate
        // partial catalog, not a physics failure. Skip loudly rather than panic.
        eprintln!("no 99942 Apophis table present — skipping the capstone");
        return;
    };

    let checks: Vec<usize> = (1..=CHECK_YEARS)
        .map(|y| SEED_INDEX + SAMPLES_PER_YEAR * y)
        .collect();
    let i_end = *checks.last().expect("at least one checkpoint");
    assert!(
        i_end < 3200,
        "arc reaches sample {i_end}, into the 2029 flyby — pick an earlier window"
    );

    // Seed: JPL's own heliocentric state at SEED_INDEX, lifted into SSB through
    // the single Sun-SSB axis.
    let epoch0 = Epoch::from_tdb_seconds_past_j2000(apophis.sample_epoch_tdb(SEED_INDEX));
    let sun0 = sun_ssb(&eph, epoch0);
    let helio0 = apophis.sample(SEED_INDEX).expect("seed sample");
    let seed = StateVector::new(
        helio0.position + sun0.position,
        helio0.velocity + sun0.velocity,
    );

    // Three fields on one DE440 point-mass base. `tier1` rebuilds it so each field
    // owns its own terms (CompositeForce::with consumes self).
    let tier1 = || CompositeForce::new().with(Box::new(tier1_perturber_field(&eph).unwrap()));
    let sun_term = || EphemerisPerturber::new(Arc::clone(&eph), SUN_J2000);
    let field_t1 = tier1();
    let field_gr = tier1().with(Box::new(Relativity1PN::new(mu_sun, sun_term())));
    let field_yk = tier1()
        .with(Box::new(Relativity1PN::new(mu_sun, sun_term())))
        .with(Box::new(YarkovskyA2::standard(APOPHIS_A2_SI, sun_term())));

    // Tight integrator so integration error (~0.1 m over the arc) stays far below
    // the km-scale physics residual we are measuring. Same stepper for all three:
    // the difference between the curves is the force model, nothing else.
    let dop = Dop853::new().with_tolerances(1e-12, 1e-6);
    let cadence = 4.0 * DAY_S;
    let n = ((i_end - SEED_INDEX) as f64 / 4.0).ceil() as u32 + 1;
    let fly = |f: &CompositeForce| {
        Clock::propagate(&dop, f, epoch0, seed, cadence, n).expect("integration")
    };
    let (c_t1, c_gr, c_yk) = (fly(&field_t1), fly(&field_gr), fly(&field_yk));

    // Residual against JPL's raw held-out states at each checkpoint.
    let mut r_t1 = Vec::new();
    let mut r_gr = Vec::new();
    let mut r_yk = Vec::new();
    eprintln!("yr   Tier-1(km)   +GR(km)   +GR+Yk(km)   GR factor   Yk helps by");
    for (y, &idx) in checks.iter().enumerate() {
        let epoch = Epoch::from_tdb_seconds_past_j2000(apophis.sample_epoch_tdb(idx));
        let sun = sun_ssb(&eph, epoch);
        let truth = apophis.sample(idx).expect("held-out sample").position;
        let resid = |c: &Clock| {
            let ssb = c.state_at(epoch).expect("checkpoint inside span").position;
            ((ssb - sun.position) - truth).norm() / 1000.0
        };
        let (a, b, d) = (resid(&c_t1), resid(&c_gr), resid(&c_yk));
        eprintln!(
            "{:<4} {a:>9.3} {b:>9.3} {d:>11.3} {:>10.1}x {:>+12.3}",
            y + 1,
            a / b,
            b - d,
        );
        r_t1.push(a);
        r_gr.push(b);
        r_yk.push(d);
    }

    // --- Baseline: there is a real signal to reduce -------------------------
    // Without the missing physics the trajectory drifts hundreds of km off JPL.
    assert!(
        *r_t1.last().unwrap() > 80.0,
        "Tier-1-only residual at the arc end is only {:.1} km — too small for the \
         term-on reductions below to mean anything",
        r_t1.last().unwrap()
    );

    // --- 1PN relativity: the headline, robust at every epoch ----------------
    for (y, (&t1, &gr)) in r_t1.iter().zip(&r_gr).enumerate() {
        assert!(
            gr < 0.5 * t1,
            "year {}: +GR residual {gr:.2} km is not below half of Tier-1's {t1:.2} km \
             — relativity should pull the trajectory materially toward JPL",
            y + 1
        );
    }
    // Order-of-magnitude early, where the along-track floor has not yet grown.
    for y in 0..3 {
        assert!(
            r_t1[y] / r_gr[y] > 6.0,
            "year {}: +GR only reduces the residual {:.1}x — relativity is an \
             order-of-magnitude effect on this arc",
            y + 1,
            r_t1[y] / r_gr[y]
        );
    }
    // And it stays bounded in absolute terms — no late blow-up.
    assert!(
        *r_gr.last().unwrap() < 80.0,
        "+GR residual reaches {:.1} km — larger than measured, integration or field \
         has regressed",
        r_gr.last().unwrap()
    );

    // --- Yarkovsky: a secular signal that clears the floor over the baseline -
    // Early it sits at/below the floor (its along-track drift is still tiny); by
    // the end of the arc the real A2 measurably reduces the residual against JPL.
    let (gr_end, yk_end) = (*r_gr.last().unwrap(), *r_yk.last().unwrap());
    assert!(
        yk_end < gr_end - 3.0,
        "at the arc end Yarkovsky (A2 = {APOPHIS_A2_SI:.3e} m/s²) leaves the residual at \
         {yk_end:.1} km vs GR-only {gr_end:.1} km — the real transverse non-grav should \
         reduce it once its secular signal has grown"
    );
}

/// SSB-relative Sun state (m, m/s) — the single conversion axis shared by seeding
/// (`helio + sun`) and comparison (`ssb − sun`), so the Sun's barycentric wobble
/// cancels rather than leaking into the residual (the frame hazard, made one line).
fn sun_ssb(eph: &Arc<Ephemeris>, epoch: Epoch) -> StateVector {
    EphemerisPerturber::new(Arc::clone(eph), SUN_J2000)
        .state_at(epoch)
        .expect("sun ssb state")
}
