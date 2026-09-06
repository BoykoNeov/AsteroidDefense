//! Is `dop853` **converged** on the arcs this project now flies — and if not, is
//! the limiting factor the tolerance, the snapshot cadence, or the method?
//!
//! HANDOFF's roadmap carries "dop853 → IAS15 crossover" as an open item, on the
//! grounds that the keyhole returns put 15-year multi-revolution arcs in the
//! pipeline. Before building a second integrator, three things have to be true:
//! that the numerical error is *material* at the tolerance we ship, that it is the
//! **method's** error rather than the architecture's, and that there is something
//! to validate a new method against. This probe settles the first two by
//! measurement. It cannot settle the third, and that is itself the finding — see
//! the module note at the bottom.
//!
//! # What the shipping tolerance actually allows
//!
//! `Dop853`'s error scale is `atol + rtol·|y|` per component. The forward
//! propagations all ran at `Dop853::new()`'s default `rtol = atol = 1e-9`, in SI,
//! where a heliocentric position component is ~`1.5e11 m`. So the controller
//! accepts a sub-step whose estimated local position error is up to **~150 m**,
//! and `atol` never binds on anything (`1e-9 m` against a `150 m` relative term).
//! Meanwhile the *backward* seed design runs at `back_rtol = 1e-12` and every
//! force-term isolation test uses `1e-12`/`1e-13`. The forward path — the one every
//! published number in this project comes out of — is the loosest thing in the
//! crate. Whether 150 m per step accumulates into anything that matters is what
//! gets measured here rather than argued about.
//!
//! # Three candidate answers, so three things are separated
//!
//! 1. **Tolerance.** Sweep `forward_rtol` over `1e-9 … 1e-13` and read the
//!    quantities the project's claims rest on. The tightest run is the reference;
//!    `campaign` checks it *is* one by watching successive differences fall
//!    geometrically (if they stop falling, roundoff has been reached and
//!    tightest-as-reference is no longer trustworthy — see [`geometric_fall`]).
//! 2. **Architecture.** [`Clock::propagate`] restarts the adaptive controller once
//!    per snapshot, so a 1-day cadence restarts it ~4 400 times across the
//!    campaign, and `probe_tier3_cost` already measured the cadence moving the
//!    b-plane perigee (+118 m at 10 days, +13.6 km at 30). The `campaign` mode
//!    therefore flies the same span **twice**: once as a single uninterrupted
//!    `step` call, once through the shipping clock path. The clean run is the
//!    integrator's own convergence; the gap between them at equal tolerance is
//!    what the snapshot architecture costs.
//! 3. **Method.** Only reachable if 1 and 2 leave a residual that matters.
//!
//! # The observables are the claims, not "max position difference"
//!
//! A converged end state is not the point — the point is whether any *conclusion*
//! moves. So `keyhole` reports the four numbers the keyhole batch's reasoning
//! actually reads, at each tolerance:
//!
//! - **`ζ₂`**, the return's timing coordinate. This is the load-bearing one. The
//!   refined 3:4 floor is called converged *because* `ζ₂` is small beside `ξ₂` —
//!   and truncation error over a 15-year arc lands as secular **phase** error,
//!   which is exactly `ζ₂`. If tolerance moves `ζ₂` by hundreds of km, that check
//!   dissolves and so does the door-width calibration built on it.
//!
//!   **What `ζ₂` reads here is not the floor's `ζ₂`** (settled 2026-09-06). This
//!   probe flies [`DV_FLOOR_M_S`], a *six-decimal* Δv, and `ζ₂` responds at 5.4e8
//!   km per m/s — so the rounding alone is ±271 km of it. The table below prints
//!   ~891 km; the true refined floor (`0.2165483096`, 20 iterations) reads −26.6
//!   km. Both are honest flights of different shots. This probe is unaffected
//!   either way: it measures how much `ζ₂` **moves with tolerance at a fixed Δv**,
//!   and 365 m is 365 m wherever on the curve you stand. Only the *denominator* in
//!   "365 m out of 891 km" was an artefact — against the ~25 km closed-form keyhole
//!   width, the honest scale for "does this change a conclusion", it is 1.5 %.
//! - **`ξ₂`**, the spatial floor no timing change removes.
//! - the return distance, which the golden-section search minimises.
//! - encounter 1's perigee, which every Tier-2 term was measured as a shift in.
//!
//! # Modes
//!
//! ```text
//!   cargo run -p asteroid_core --release --example probe_integrator_convergence -- <mode>
//! ```
//!
//! - `determinism` (2 builds, ~30 s) — **run this first.** Two identical-tolerance
//!   re-flies must agree bit-for-bit. Every delta the other modes print is a
//!   difference of two runs, so a nondeterministic re-fly makes all of them
//!   unreadable. Same gate that made Pluto's 0.6 m readable as signal.
//! - `campaign` (5 tolerances × ~4 flights, ~5 min) — the 12-year campaign, clean
//!   path vs clock path, plus a forward-then-back reversibility residual that needs
//!   no reference run at all.
//! - `cadence` (3 builds × 4 propagations, ~3 min) — tolerance crossed against
//!   snapshot cadence on the encounter perigee. Run this **after** `campaign`: it
//!   exists to test the explanation `campaign`'s numbers suggest, and it is the
//!   mode that decides which of the three candidates above is the answer.
//! - `keyhole` (5 tolerances × 1 flight, ~4 min) — the 15-year 3:4 keyhole return
//!   at the refined Δv, in the coordinates the conclusions are stated in.
//! - `all` — determinism, campaign, cadence, keyhole.
//!
//! # Why the third candidate cannot be settled here, which is the finding
//!
//! "Is IAS15 better than dop853 on **our** field" has no oracle, and that is a
//! stronger reason not to build it than any cost estimate. HANDOFF §6 nominates
//! REBOUND for the comparison and in the same breath rules it out as a *trajectory*
//! oracle, because REBOUND self-gravitates the planets while we fly a test particle
//! in an ephemeris field — it is good for free invariants and encounter sensitivity,
//! not for "whose position is right". Horizons cannot stand in either:
//! [`horizons`](asteroid_core::horizons) documents that a 1-day state table cannot
//! resolve a flyby of this class (18 885 km of its own interpolation error), and a
//! resonant return three years past a *deflection we invented* is in no truth table
//! anywhere.
//!
//! That leaves dop853-at-a-tighter-tolerance as the only available oracle — which is
//! exactly what the modes below run. So if self-convergence says dop853 is converged,
//! a second integrator has nothing to prove against, and building it would be
//! unfalsifiable work. Hence: the roadmap item is **retired, not deferred**.
//!
//! Requires kernels.

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, closest_approach, fly_keyhole_shot, EphemerisPerturber, ImpactorConfig,
    Integrator, KeyholeAiming, KeyholeShot, KeyholeShotOptions, OpikFrame, RealFieldScenario,
    Resonance, ScanOptions, StateVector,
};
use std::sync::Arc;
use std::time::Instant;

/// The forward relative tolerances swept, loosest (shipping) first.
///
/// Stops at `1e-13` deliberately. At 1 AU that is `rtol·|r| ≈ 1.5 cm` against a
/// double's ~`2e-5 m` resolution on `1.5e11` — three decades of headroom, so the
/// controller is still asking for something achievable. Tightening further asks
/// for error below what the arithmetic can represent, at which point the "reference"
/// run is measuring roundoff and every delta against it is noise.
const RTOLS: [f64; 5] = [1.0e-9, 1.0e-10, 1.0e-11, 1.0e-12, 1.0e-13];

/// The refined floor of the shipping rock's 3:4 keyhole, m/s — measured by
/// `probe_keyhole_return` at the **shipping** tolerance on 2026-09-06.
///
/// **Rounded to six decimals, and deliberately left that way.** The real
/// 20-iteration floor is `0.2165483096`; this constant is 1.7e-6 m/s off it, which
/// is ~917 km of `ζ₂` at 5.4e8 km per m/s. Changing it would move every absolute
/// number in the table without changing a single *delta*, since what this probe
/// measures is the tolerance derivative at a fixed shot. Keeping it pins the
/// published sweep. Just never read the absolute `ζ₂` below as the floor's `ζ₂` —
/// that mistake is what made this file's own header contradict its own table for
/// two months.
///
/// Held fixed across the sweep on purpose. Re-solving it per tolerance would cost
/// four minutes each and would answer a different question: this probe asks whether
/// the *same* shot reads the same at a tighter tolerance, which is what says whether
/// the published numbers are converged. (If the shot itself moves, the solve moves
/// too — but then the published floor was already tolerance-dependent, which is the
/// finding.)
const DV_FLOOR_M_S: f64 = 0.216_550;

/// How far short of the encounter the un-amplified comparison is taken, years.
///
/// The campaign span ends 60 days *past* a 3 000 km Earth flyby, and a flyby that
/// close multiplies whatever error arrives at it — so an end-state difference is a
/// method error times an amplification factor nobody has measured, and quoting it
/// as "the integrator is off by X" would be quoting the wrong number. Backing off
/// a year puts the comparison on the cruise, where nothing is amplifying anything,
/// and the two columns side by side show how much of the end-state figure was the
/// flyby rather than the integration.
const PRE_ENCOUNTER_BACKOFF_YEARS: f64 = 1.0;

/// The resonance flown — the one the keyhole conclusions were measured on.
const RESONANCE: Resonance = Resonance { h: 3, k: 4 };

fn main() {
    let mode = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "determinism".to_string())
        .to_lowercase();

    match mode.as_str() {
        "determinism" => determinism(),
        "campaign" => campaign(),
        "cadence" => cadence(),
        "keyhole" => keyhole(),
        "all" => {
            determinism();
            campaign();
            cadence();
            keyhole();
        }
        other => {
            eprintln!(
                "unknown mode {other:?}; use determinism | campaign | cadence | keyhole | all"
            );
            std::process::exit(2);
        }
    }
}

/// Build the shipping scenario with one forward tolerance changed.
///
/// `back_rtol` is untouched, so the **seed is the same state in every run** — only
/// the forward field integration differs. `campaign` asserts that rather than
/// trusting it; without it a "tolerance sweep" would be sweeping the seed too and
/// the deltas would mean nothing.
fn build(forward_rtol: f64) -> RealFieldScenario {
    let cfg = ImpactorConfig {
        forward_rtol,
        ..ImpactorConfig::default()
    };
    match RealFieldScenario::build(&cfg) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed at rtol {forward_rtol:.0e}: {e}");
            eprintln!("(this probe needs DE440 kernels — see kernels::resolve)");
            std::process::exit(1);
        }
    }
}

// --- Gate: two identical runs must agree bit-for-bit ------------------------

/// Two builds at the *same* tolerance, compared bit-for-bit.
///
/// This is the gate, not a formality. Every number the other modes print is a
/// difference between two propagations; if a re-fly at fixed inputs can wander,
/// a tolerance delta and a nondeterminism are indistinguishable and the whole
/// batch is unreadable. `HANDOFF` §2 already promises same-build-same-output (it
/// declines only *cross-machine* bit-identity), and the Pluto measurement leaned
/// on exactly this identity to read 0.6 m as physics.
fn determinism() {
    println!("=== determinism gate: two identical-tolerance runs, bit-for-bit ===");
    let rtol = RTOLS[0];
    let t = Instant::now();
    let a = build(rtol);
    let b = build(rtol);
    println!(
        "two builds at rtol {rtol:.0e} in {:.1} s",
        t.elapsed().as_secs_f64()
    );

    let seed_same = a.seed() == b.seed();
    println!(
        "  seed state            : {}",
        if seed_same {
            "identical (bit-for-bit)"
        } else {
            "DIFFERENT — the backward design is not deterministic"
        }
    );

    let (pa, pb) = (campaign_perigee(&a), campaign_perigee(&b));
    let same = match (pa, pb) {
        (Some(x), Some(y)) => x.to_bits() == y.to_bits(),
        _ => false,
    };
    println!(
        "  encounter-1 perigee   : {} / {}  → {}",
        fmt_opt_km(pa),
        fmt_opt_km(pb),
        if same {
            "identical (bit-for-bit)"
        } else {
            "DIFFERENT"
        }
    );

    let (ea, eb) = (clock_end_state(&a), clock_end_state(&b));
    let end_same = match (&ea, &eb) {
        (Some(x), Some(y)) => bits_equal(x, y),
        _ => false,
    };
    println!(
        "  end state (span end)  : {}",
        if end_same {
            "identical (bit-for-bit)"
        } else {
            "DIFFERENT"
        }
    );

    if seed_same && same && end_same {
        println!("\nGATE PASSED — deltas printed by the other modes are readable as signal.");
    } else {
        println!(
            "\nGATE FAILED — stop here. A tolerance delta and a nondeterministic re-fly are\n\
             indistinguishable until this passes, so nothing measured downstream means anything."
        );
        std::process::exit(1);
    }
}

// --- The 12-year campaign: tolerance, and the cadence's share of it ---------

/// One tolerance's worth of campaign measurements.
struct CampaignRow {
    rtol: f64,
    /// End state of one uninterrupted `step` call over the whole span.
    clean: Option<StateVector>,
    /// End state of the same span through `Clock::propagate` at shipping cadence.
    clock: Option<StateVector>,
    /// Clean end state a year *short* of the encounter — the same method error with
    /// the flyby's amplification removed. See [`PRE_ENCOUNTER_BACKOFF_YEARS`].
    clean_pre: Option<StateVector>,
    /// The clock path at that same pre-encounter epoch, which is a snapshot
    /// boundary by construction (so this reads the integration, not the interpolant).
    clock_pre: Option<StateVector>,
    /// Encounter-1 perigee off the clock path, m — the project's own observable.
    perigee_m: Option<f64>,
    /// `|r|` residual of forward-then-back on the clean path, m.
    reversibility_m: Option<f64>,
    clean_seconds: f64,
    clock_seconds: f64,
}

fn campaign() {
    println!("\n=== campaign: 12-year arc, clean integration vs the shipping clock path ===");
    let cfg = ImpactorConfig::default();
    let span = cfg.lead_years * 365.25 * 86_400.0 + cfg.span_margin_days * 86_400.0;
    println!(
        "span {:.2} yr, cadence {:.0} d, {} tolerances; shipping is {:.0e}",
        span / (365.25 * 86_400.0),
        cfg.cadence_days,
        RTOLS.len(),
        RTOLS[0]
    );
    println!(
        "the clean path is ONE adaptive integration; the clock path restarts the controller\n\
         at every snapshot. Their gap at equal tolerance is what the cadence costs."
    );

    let mut rows: Vec<CampaignRow> = Vec::new();
    let mut seed0: Option<StateVector> = None;
    for rtol in RTOLS {
        let s = build(rtol);
        // The seed must not move with the forward tolerance — otherwise this sweep
        // is not measuring the forward integration at all.
        match &seed0 {
            None => seed0 = Some(s.seed()),
            Some(first) => assert!(
                bits_equal(first, &s.seed()),
                "seed moved with forward_rtol — the sweep is not isolating the forward path"
            ),
        }

        let epoch0 = s.epoch0();
        let seed = s.seed();
        let stepper = s.forward_stepper();

        let t = Instant::now();
        let clean = stepper.step(s.field(), epoch0, &seed, span).ok();
        let clean_seconds = t.elapsed().as_secs_f64();

        // Forward-then-back on the clean path: a free invariant, needing no
        // reference run. Error can cancel on the return leg, so this bounds
        // accumulated truncation from below rather than measuring it exactly —
        // which is enough to see it fall by decades, or fail to.
        let reversibility_m = clean.and_then(|end| {
            let back = stepper
                .step(s.field(), epoch0.shifted_by_seconds(span), &end, -span)
                .ok()?;
            Some((back.position - seed.position).norm())
        });

        let t = Instant::now();
        let clock = clock_end_state(&s);
        let clock_seconds = t.elapsed().as_secs_f64();

        // The un-amplified pair. Snapped to a whole number of snapshots so the
        // clock query lands on a boundary and reads the integration rather than
        // the dense interpolant between two of them.
        let cadence = s.cadence_seconds();
        let n_pre = ((span - PRE_ENCOUNTER_BACKOFF_YEARS * 365.25 * 86_400.0) / cadence).floor();
        let pre_span = n_pre * cadence;
        let clean_pre = stepper.step(s.field(), epoch0, &seed, pre_span).ok();
        let clock_pre = s
            .propagate_free(epoch0, seed, cadence, n_pre as u32)
            .ok()
            .and_then(|c| c.state_at(epoch0.shifted_by_seconds(pre_span)).ok());

        let perigee_m = campaign_perigee(&s);
        rows.push(CampaignRow {
            rtol,
            clean,
            clock,
            clean_pre,
            clock_pre,
            perigee_m,
            reversibility_m,
            clean_seconds,
            clock_seconds,
        });
    }

    let reference = rows.last().expect("one row per tolerance");
    println!(
        "\nreference = the tightest run, rtol {:.0e}. All Δ are against it.",
        reference.rtol
    );
    println!(
        "\n  {:>7}  {:>13}  {:>13}  {:>13}  {:>13}  {:>13}  {:>7}  {:>7}",
        "rtol",
        "clean Δ|r| m",
        "clock Δ|r| m",
        "clean↔clock m",
        "perigee km",
        "Δperigee m",
        "clean s",
        "clock s"
    );
    for row in &rows {
        let clean_d = delta_position(row.clean.as_ref(), reference.clean.as_ref());
        let clock_d = delta_position(row.clock.as_ref(), reference.clock.as_ref());
        let gap = delta_position(row.clean.as_ref(), row.clock.as_ref());
        let dp = match (row.perigee_m, reference.perigee_m) {
            (Some(a), Some(b)) => Some(a - b),
            _ => None,
        };
        let designed = row.perigee_m.map(|p| p - cfg.b_offset_km * 1.0e3);
        println!(
            "  {:>7.0e}  {:>13}  {:>13}  {:>13}  {:>13}  {:>13}  {:>12}  {:>7.2}  {:>7.2}",
            row.rtol,
            fmt_opt_e(clean_d),
            fmt_opt_e(clock_d),
            fmt_opt_e(gap),
            fmt_opt_km(row.perigee_m),
            fmt_opt_e(dp),
            designed
                .map(|d| format!("{d:+.3}"))
                .unwrap_or_else(|| "-".into()),
            row.clean_seconds,
            row.clock_seconds,
        );
    }

    println!(
        "\n  the same two paths a year SHORT of the flyby, so nothing is amplified — this is\n  \
         the method's own error on the cruise, and `amplif.` is how much of the end-state\n  \
         column above was the encounter multiplying it rather than the integration:"
    );
    println!(
        "\n  {:>7}  {:>13}  {:>13}  {:>13}  {:>11}",
        "rtol", "clean \u{394}|r| m", "clock \u{394}|r| m", "clean\u{2194}clock m", "amplif."
    );
    for row in &rows {
        let pre = delta_position(row.clean_pre.as_ref(), reference.clean_pre.as_ref());
        let pre_clock = delta_position(row.clock_pre.as_ref(), reference.clock_pre.as_ref());
        let pre_gap = delta_position(row.clean_pre.as_ref(), row.clock_pre.as_ref());
        let end = delta_position(row.clean.as_ref(), reference.clean.as_ref());
        let amp = match (end, pre) {
            (Some(e), Some(p)) if p > 0.0 => Some(e / p),
            _ => None,
        };
        println!(
            "  {:>7.0e}  {:>13}  {:>13}  {:>13}  {:>11}",
            row.rtol,
            fmt_opt_e(pre),
            fmt_opt_e(pre_clock),
            fmt_opt_e(pre_gap),
            amp.map(|a| format!("{a:.1}x"))
                .unwrap_or_else(|| "-".into()),
        );
    }

    println!("\n  reversibility (forward {span:.3e} s then back, clean path):");
    for row in &rows {
        println!(
            "    rtol {:>7.0e}  |Δr| {:>13}",
            row.rtol,
            fmt_opt_e(row.reversibility_m)
        );
    }

    // Is the reference converged? Successive rungs must close geometrically. If
    // the last gap is not much smaller than the one before it, the sweep has hit
    // roundoff and "tightest as reference" stops being a valid claim.
    let clean_ladder: Vec<Option<f64>> = rows
        .windows(2)
        .map(|w| delta_position(w[0].clean.as_ref(), w[1].clean.as_ref()))
        .collect();
    println!("\n  successive-rung differences (clean path), and whether they are still falling:");
    for (w, d) in rows.windows(2).zip(&clean_ladder) {
        println!(
            "    {:>7.0e} → {:>7.0e}  |Δr| {:>13}",
            w[0].rtol,
            w[1].rtol,
            fmt_opt_e(*d)
        );
    }
    match geometric_fall(&clean_ladder) {
        Some(true) => println!(
            "    → still falling geometrically at the tight end: the reference is converged\n      \
             and every Δ above is a real tolerance effect."
        ),
        Some(false) => println!(
            "    → NOT falling at the tight end: this sweep has reached roundoff, so the\n      \
             tightest run is not a valid reference and the smallest Δ above are noise."
        ),
        None => println!("    → not enough finite rungs to judge."),
    }
}

/// Encounter 1's perigee on the shipping clock path, m.
fn campaign_perigee(s: &RealFieldScenario) -> Option<f64> {
    let ds = s.deflection().ok()?;
    let enc = s.nominal_hit(&ds).ok()?;
    Some(enc.perigee)
}

/// The state at the end of the shipping-cadence clock propagation.
///
/// Queried at the final **snapshot boundary**, where the dense interpolant meets
/// the accepted step (`dense_output_endpoints_match_the_step`), so this reads the
/// integration's own answer and not the interpolant's error.
fn clock_end_state(s: &RealFieldScenario) -> Option<StateVector> {
    let n = s.n_snapshots();
    let cadence = s.cadence_seconds();
    let clock = s.propagate_free(s.epoch0(), s.seed(), cadence, n).ok()?;
    clock
        .state_at(s.epoch0().shifted_by_seconds(cadence * f64::from(n)))
        .ok()
}

// --- Crossing the two axes: is accuracy set by the tolerance or by the cadence? --

/// Snapshot cadences crossed against tolerance, days. `1` is shipping, `10` is what
/// Tier-3 samples at, `30` is where `probe_tier3_cost` measured the perigee moving
/// 13.6 km, and the last two are past any cadence this project uses — carried because
/// the interesting question turned out to be how coarse a cadence a *tight* tolerance
/// can carry, and that has to be asked past the point where it breaks.
const CADENCE_DAYS: [f64; 6] = [1.0, 3.0, 10.0, 30.0, 90.0, 180.0];

/// One cell of the cadence x tolerance cross: the cadence in days, the perigee it
/// reached (`None` if no close approach fell inside the gate), and its wall clock.
type CrossCell = (f64, Option<f64>, f64);

/// One tolerance's row of that cross — the tolerance, then a cell per cadence.
type CrossRow = (f64, Vec<CrossCell>);

/// The scan sampling step the whole project reduces encounter 1 with, seconds.
const SHIPPING_SAMPLE_DT: f64 = 6.0 * 3600.0;

/// Scan sampling steps for the residual study — the shipping one, then finer.
const SCAN_SAMPLE_DTS: [f64; 3] = [6.0 * 3600.0, 3600.0, 900.0];

/// The cadence Tier-3 covariance sampling runs at, days (`SAMPLE_CADENCE_DAYS`).
const TIER3_CADENCE_DAYS: f64 = 10.0;

/// The fine-cadence baseline of the residual study, days — 3 rather than 1 because
/// the cross table has them agreeing to 0.022 m at a third of the cost.
const FINE_CADENCE_DAYS: f64 = 3.0;

/// Finite-difference step for the derivative column, m/s — the same one
/// `probe_tier3_cost` measured its columns with, so the two are comparable.
const FD_STEP_M_S: f64 = 1.0e-4;

/// Tolerances for the cross, coarse enough to stay affordable (one build each).
const CROSS_RTOLS: [f64; 3] = [1.0e-9, 1.0e-11, 1.0e-13];

/// The experiment that decides which of the three candidate answers is right.
///
/// `campaign` turns up something backwards: the **clock** path, which restarts the
/// controller at every snapshot, tracks the tightest reference far better than one
/// uninterrupted integration at the same tolerance — and its residual barely
/// responds to tolerance at all. The natural reading is that a snapshot boundary
/// **caps the step size**, and that the cap, not the error tolerance, is what
/// actually sets the shipping accuracy. If so, the tolerance is not binding at
/// 1-day cadence and tightening it buys nothing.
///
/// That reading makes a falsifiable prediction, which is why it is worth a mode
/// rather than a paragraph. `probe_tier3_cost` measured the perigee moving
/// **+13.6 km** when the cadence coarsens to 30 days, at the shipping tolerance.
/// If that shift is a step-size effect, then at a tight tolerance the controller
/// must subdivide *inside* the 30-day interval and the shift has to **shrink**.
/// If it survives at `1e-13`, it is not step size at all and something else in the
/// coarse-cadence path is wrong — the dense output the scan samples, most likely,
/// which would be a different bug in a different place.
///
/// Either way the answer arrives as a number, and either way it is about the
/// architecture rather than the method.
fn cadence() {
    println!("\n=== cadence x tolerance: which one sets the encounter accuracy? ===");
    let cfg = ImpactorConfig::default();
    let span = cfg.lead_years * 365.25 * 86_400.0 + cfg.span_margin_days * 86_400.0;
    println!(
        "encounter-1 perigee, {} cadences x {} tolerances. The prediction under test: if the\n\
         snapshot cap is what sets accuracy, the coarse-cadence shift must SHRINK as rtol tightens.",
        CADENCE_DAYS.len(),
        CROSS_RTOLS.len()
    );

    let mut table: Vec<CrossRow> = Vec::new();
    let mut built: Vec<(f64, RealFieldScenario)> = Vec::new();
    for rtol in CROSS_RTOLS {
        let s = build(rtol);
        let mut row = Vec::new();
        for days in CADENCE_DAYS {
            let t = Instant::now();
            let perigee = perigee_at(&s, days * 86_400.0, span, SHIPPING_SAMPLE_DT, None);
            row.push((days, perigee, t.elapsed().as_secs_f64()));
        }
        table.push((rtol, row));
        built.push((rtol, s));
    }

    println!("\n  perigee, km:");
    print!("  {:>7}", "rtol");
    for days in CADENCE_DAYS {
        print!("  {:>16}", format!("{days:.0} d"));
    }
    println!();
    for (rtol, row) in &table {
        print!("  {rtol:>7.0e}");
        for (_, perigee, _) in row {
            print!("  {:>16}", fmt_opt_km_6(*perigee));
        }
        println!();
    }

    println!("\n  shift vs that row's own 1-day cell, metres — the coarse-cadence penalty:");
    print!("  {:>7}", "rtol");
    for days in CADENCE_DAYS {
        print!("  {:>16}", format!("{days:.0} d"));
    }
    println!();
    for (rtol, row) in &table {
        let base = row.first().and_then(|c| c.1);
        print!("  {rtol:>7.0e}");
        for (_, perigee, _) in row {
            let d = match (perigee, base) {
                (Some(p), Some(b)) => Some(p - b),
                _ => None,
            };
            print!(
                "  {:>16}",
                d.map(|d| format!("{d:+.3}")).unwrap_or_else(|| "-".into())
            );
        }
        println!();
    }

    println!("\n  wall clock per propagation + scan, s:");
    for (rtol, row) in &table {
        print!("  {rtol:>7.0e}");
        for (_, _, secs) in row {
            print!("  {secs:>16.2}");
        }
        println!();
    }

    // The verdict, read off the coarse column rather than asserted.
    let loose = cross_penalty(table.first());
    let tight = cross_penalty(table.last());
    match (loose, tight) {
        (Some(l), Some(t)) if l.abs() > 0.0 => {
            let ratio = t.abs() / l.abs();
            println!(
                "\n  the {:.0}-day penalty is {:+.1} m at rtol {:.0e} and {:+.1} m at {:.0e} ({ratio:.3}x).",
                CADENCE_DAYS[CADENCE_DAYS.len() - 1],
                l,
                CROSS_RTOLS[0],
                t,
                CROSS_RTOLS[CROSS_RTOLS.len() - 1],
            );
            if ratio < 0.2 {
                println!(
                    "  -> it SHRANK with tolerance. The coarse-cadence error was step size all\n  \
                     along, so at 1-day cadence the snapshot cap — not rtol — is what sets the\n  \
                     shipping accuracy, and tightening rtol at 1 day buys nothing."
                );
            } else {
                println!(
                    "  -> it SURVIVED a tight tolerance. Then it is not step size, and the\n  \
                     coarse-cadence error lives somewhere else in that path (the dense output\n  \
                     the scan samples is the first place to look)."
                );
            }
        }
        _ => println!("\n  not enough finite cells to judge."),
    }

    derivative_study(&built, span);
    scan_resolution_study(&built, span);
}

/// Does the cadence bias survive in a **derivative**, which is what Tier 3 reads?
///
/// The table above is the *nominal* perigee: one seed, no impulse. Tier 3 never
/// consumes that. It consumes finite-difference columns, and HANDOFF's standing
/// argument is that a same-cadence difference cancels the cadence's systematic
/// error to first order (`probe_tier3_cost` measured the column holding to 0.024 %
/// at 10 days while the absolute perigee moved 118 m). So the 117.7 m the table
/// shows at 10 days is a bias in a number Tier 3 does not read, and "tighten rtol
/// to remove it" would be recommending a fix for a non-problem.
///
/// One column settles it: `∂(perigee)/∂v_along` at the Tier-3 cadence, at both ends
/// of the tolerance range. If the derivative moves as little as the cancellation
/// argument predicts, the absolute-placement finding stays an absolute-placement
/// finding and says nothing about Tier-3 sampling.
fn derivative_study(built: &[(f64, RealFieldScenario)], span: f64) {
    println!(
        "\n  --- does the bias survive a DERIVATIVE? (the only thing Tier 3 reads) ---\n  \
         one central-difference column, d(perigee)/dv_along, h = {:.0e} m/s, at the Tier-3\n  \
         sampling cadence of {:.0} days:",
        FD_STEP_M_S, TIER3_CADENCE_DAYS
    );
    println!(
        "\n  {:>7}  {:>18}  {:>16}  {:>14}",
        "rtol", "d(perigee)/dv_along", "vs tightest", "as fraction"
    );
    let cadence = TIER3_CADENCE_DAYS * 86_400.0;
    let columns: Vec<(f64, Option<f64>)> = built
        .iter()
        .map(|(rtol, s)| {
            let along = along_track_unit(s.seed()).expect("along-track");
            let plus = perigee_at(
                s,
                cadence,
                span,
                SHIPPING_SAMPLE_DT,
                Some(FD_STEP_M_S * along),
            );
            let minus = perigee_at(
                s,
                cadence,
                span,
                SHIPPING_SAMPLE_DT,
                Some(-FD_STEP_M_S * along),
            );
            let d = match (plus, minus) {
                (Some(p), Some(m)) => Some((p - m) / (2.0 * FD_STEP_M_S)),
                _ => None,
            };
            (*rtol, d)
        })
        .collect();
    let reference = columns.last().and_then(|c| c.1);
    for (rtol, d) in &columns {
        let delta = match (d, reference) {
            (Some(d), Some(r)) => Some(d - r),
            _ => None,
        };
        let frac = match (delta, reference) {
            (Some(dd), Some(r)) if r != 0.0 => Some(dd / r),
            _ => None,
        };
        println!(
            "  {rtol:>7.0e}  {:>18}  {:>16}  {:>14}",
            d.map(|d| format!("{d:.6e}")).unwrap_or_else(|| "-".into()),
            delta
                .map(|d| format!("{d:+.3e}"))
                .unwrap_or_else(|| "-".into()),
            frac.map(|f| format!("{:+.4} %", f * 100.0))
                .unwrap_or_else(|| "-".into()),
        );
    }
    println!(
        "  a fraction of a percent here means the cancellation argument holds and the\n  \
         117.7 m above is absolute placement only — NOT a Tier-3 sampling error."
    );
}

/// Where does the sub-metre residual that survives a tight tolerance come from?
///
/// At `rtol = 1e-13` the perigee still walks monotonically with cadence — 0.36 m at
/// 10 days, 0.51 at 30, 0.66 at 90, 0.68 at 180 — growing and then saturating. That
/// shape is not scan noise; it is a systematic that tracks snapshot spacing. The
/// candidate is the degree-7 dense interpolant `closest_approach` samples: its
/// per-segment accuracy degrades as the accepted sub-steps it spans grow, and coarse
/// snapshots are exactly what lets them grow.
///
/// The discriminator is the scan's own sampling step, which is independent of both
/// the cadence and the tolerance. Halving and quartering `max_sample_dt` cannot
/// change the integration at all, so if the residual shrinks it belongs to the
/// scan/interpolant, and if it does not it belongs to the trajectory.
///
/// The fine-cadence baseline is 3 days rather than 1: the table has them agreeing to
/// 0.022 m, and 3 days costs a third as much.
fn scan_resolution_study(built: &[(f64, RealFieldScenario)], span: f64) {
    let Some((rtol, s)) = built.last() else {
        return;
    };
    println!(
        "\n  --- the sub-metre residual that a tight tolerance does NOT remove ---\n  \
         perigee at rtol {rtol:.0e}, fine cadence ({:.0} d) vs coarse ({:.0} d), as the SCAN's\n  \
         sampling step is refined. The scan cannot touch the integration, so a residual\n  \
         that shrinks here is the interpolant's and not the trajectory's:",
        FINE_CADENCE_DAYS,
        CADENCE_DAYS[CADENCE_DAYS.len() - 1]
    );
    println!(
        "\n  {:>12}  {:>16}  {:>16}  {:>14}",
        "max_sample_dt", "fine km", "coarse km", "residual m"
    );
    let fine_cadence = FINE_CADENCE_DAYS * 86_400.0;
    let coarse_cadence = CADENCE_DAYS[CADENCE_DAYS.len() - 1] * 86_400.0;
    for sample_dt in SCAN_SAMPLE_DTS {
        let fine = perigee_at(s, fine_cadence, span, sample_dt, None);
        let coarse = perigee_at(s, coarse_cadence, span, sample_dt, None);
        let residual = match (coarse, fine) {
            (Some(c), Some(f)) => Some(c - f),
            _ => None,
        };
        println!(
            "  {:>12}  {:>16}  {:>16}  {:>14}",
            format!("{:.2} h", sample_dt / 3600.0),
            fmt_opt_km_6(fine),
            fmt_opt_km_6(coarse),
            residual
                .map(|r| format!("{r:+.4}"))
                .unwrap_or_else(|| "-".into()),
        );
    }
}

/// One perigee: propagate `span` at `cadence`, scan at `sample_dt`, reduce.
///
/// `nudge` adds a velocity offset to the seed (for a finite-difference column) and
/// is `None` for the nominal. Everything else — the gate, the time tolerance, the
/// reduction constants — is the shipping scan, so a cell here is comparable to the
/// project's own perigee numbers.
fn perigee_at(
    s: &RealFieldScenario,
    cadence: f64,
    span: f64,
    sample_dt: f64,
    nudge: Option<nalgebra::Vector3<f64>>,
) -> Option<f64> {
    let ds = s.deflection().ok()?;
    let nominal = s.nominal_hit(&ds).ok()?;
    let earth = EphemerisPerturber::new(Arc::clone(s.ephemeris()), EARTH_J2000);
    let scan = ScanOptions {
        max_sample_dt: sample_dt,
        time_tol_seconds: 1.0e-3,
        max_distance: Some(5.0e8),
    };
    let seed = match nudge {
        None => s.seed(),
        Some(dv) => StateVector::new(s.seed().position, s.seed().velocity + dv),
    };
    let n = (span / cadence).ceil().max(2.0) as u32;
    let clock = s.propagate_free(s.epoch0(), seed, cadence, n).ok()?;
    let ca = closest_approach(&clock, &earth, scan).ok()??;
    ca.b_plane(nominal.mu, nominal.earth_radius)
        .ok()
        .map(|enc| enc.perigee)
}

/// The coarsest cell's perigee minus the 1-day cell's, for one tolerance row.
fn cross_penalty(row: Option<&CrossRow>) -> Option<f64> {
    let (_, cells) = row?;
    let base = cells.first()?.1?;
    let last = cells.last()?.1?;
    Some(last - base)
}

// --- The 15-year keyhole return, in the coordinates the claims use ----------

fn keyhole() {
    println!("\n=== keyhole: the 3:4 resonant return at the refined Δv, per tolerance ===");
    println!(
        "Δv held at {DV_FLOOR_M_S:.6} m/s (the floor measured at the shipping tolerance).\n\
         ζ₂ is the timing coordinate — where a 15-year arc's secular phase error would land,\n\
         and the number that says the refinement converged rather than hit a wall."
    );

    let mut rows: Vec<(f64, Option<KeyholeShot>, f64)> = Vec::new();
    for rtol in RTOLS {
        let s = build(rtol);
        let eph = s.ephemeris().clone();
        let ds = match s.deflection() {
            Ok(d) => d,
            Err(e) => {
                eprintln!("  rtol {rtol:.0e}: deflection failed: {e}");
                continue;
            }
        };
        let nominal = match s.nominal_hit(&ds) {
            Ok(n) => n,
            Err(e) => {
                eprintln!("  rtol {rtol:.0e}: no nominal hit: {e}");
                continue;
            }
        };
        let t_ca = match ds.nominal_encounter_epoch() {
            Ok(Some(t)) => t,
            _ => {
                eprintln!("  rtol {rtol:.0e}: no nominal encounter epoch");
                continue;
            }
        };
        let (r_km, v_km) = match eph.state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime()) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("  rtol {rtol:.0e}: Earth state failed: {e}");
                continue;
            }
        };
        let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
        let frame = match OpikFrame::new(&nominal, r_km * 1e3, v_km * 1e3, mu_sun) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("  rtol {rtol:.0e}: no Öpik frame: {e}");
                continue;
            }
        };
        let seed_at_0 = ds.nominal().state_at(s.epoch0()).expect("seed at epoch0");
        let direction = -along_track_unit(seed_at_0).expect("along-track");
        let aiming = KeyholeAiming {
            frame: &frame,
            earth_radius_m: nominal.earth_radius,
            deflection_epoch: s.epoch0(),
            direction,
        };
        let t = Instant::now();
        let shot = fly_keyhole_shot(
            &s,
            &ds,
            aiming,
            DV_FLOOR_M_S,
            RESONANCE,
            &KeyholeShotOptions::default(),
        )
        .ok();
        rows.push((rtol, shot, t.elapsed().as_secs_f64()));
    }

    println!(
        "\n  {:>7}  {:>13}  {:>13}  {:>11}  {:>11}  {:>8}  {:>7}",
        "rtol", "enc1 perigee", "return km", "ξ₂ km", "ζ₂ km", "|ζ₂/ξ₂|", "s"
    );
    for (rtol, shot, secs) in &rows {
        match shot {
            None => println!("  {rtol:>7.0e}  flight failed"),
            Some(shot) => {
                let r = shot.flown_return.as_ref();
                println!(
                    "  {rtol:>7.0e}  {:>13.3}  {:>13}  {:>11}  {:>11}  {:>8}  {secs:>7.1}",
                    shot.encounter.perigee / 1e3,
                    r.map(|r| format!("{:.3}", r.distance_m / 1e3))
                        .unwrap_or_else(|| "no return".into()),
                    r.and_then(|r| r.xi_m)
                        .map(|v| format!("{:.3}", v / 1e3))
                        .unwrap_or_else(|| "-".into()),
                    r.and_then(|r| r.zeta_m)
                        .map(|v| format!("{:.3}", v / 1e3))
                        .unwrap_or_else(|| "-".into()),
                    r.and_then(|r| r.timing_share())
                        .map(|v| format!("{v:.4}"))
                        .unwrap_or_else(|| "-".into()),
                );
            }
        }
    }

    // Against the tightest run, in the units the conclusions are stated in.
    let Some((ref_rtol, Some(reference), _)) = rows.last() else {
        println!("\n  no reference flight — nothing to compare.");
        return;
    };
    println!("\n  Δ against the tightest run (rtol {ref_rtol:.0e}):");
    println!(
        "  {:>7}  {:>15}  {:>13}  {:>11}  {:>11}",
        "rtol", "Δenc1 perigee m", "Δreturn km", "Δξ₂ km", "Δζ₂ km"
    );
    let rr = reference.flown_return.as_ref();
    for (rtol, shot, _) in &rows {
        let Some(shot) = shot else { continue };
        let r = shot.flown_return.as_ref();
        println!(
            "  {rtol:>7.0e}  {:>15.3}  {:>13}  {:>11}  {:>11}",
            shot.encounter.perigee - reference.encounter.perigee,
            opt_delta_km(r.map(|r| r.distance_m), rr.map(|r| r.distance_m)),
            opt_delta_km(r.and_then(|r| r.xi_m), rr.and_then(|r| r.xi_m)),
            opt_delta_km(r.and_then(|r| r.zeta_m), rr.and_then(|r| r.zeta_m)),
        );
    }
    println!(
        "\n  read ζ₂ first: the published floor's claim to be physics rather than an\n  \
         unconverged search rests on |ζ₂| ≪ |ξ₂|, and a Δζ₂ of order ζ₂ itself would\n  \
         mean the shipping tolerance, not the physics, set that number."
    );
}

// --- Small helpers ----------------------------------------------------------

/// Do two states agree in every bit? (Not `==`: this is about the exact bit
/// pattern, which is what "same build, same output" promises.)
fn bits_equal(a: &StateVector, b: &StateVector) -> bool {
    (0..3).all(|i| {
        a.position[i].to_bits() == b.position[i].to_bits()
            && a.velocity[i].to_bits() == b.velocity[i].to_bits()
    })
}

/// `|Δr|` between two states, when both exist.
fn delta_position(a: Option<&StateVector>, b: Option<&StateVector>) -> Option<f64> {
    Some((a?.position - b?.position).norm())
}

/// Are the successive-rung differences still shrinking at the tight end?
///
/// The convergence claim needs the *last* gap to be a real fraction of the one
/// before it. A ratio near or above 1 says the two tightest runs differ by about
/// as much as the two before them, which is the signature of roundoff rather than
/// truncation — at which point the tightest run is not a reference, it is just
/// another sample of the noise floor. The threshold is deliberately generous
/// (`< 0.5`): dop853 is 8th order, so a decade of tolerance should buy far more
/// than a factor of two, and anything failing this is failing badly.
fn geometric_fall(ladder: &[Option<f64>]) -> Option<bool> {
    let finite: Vec<f64> = ladder
        .iter()
        .filter_map(|d| *d)
        .filter(|d| *d > 0.0)
        .collect();
    if finite.len() < 2 {
        return None;
    }
    let last = finite[finite.len() - 1];
    let prev = finite[finite.len() - 2];
    Some(last < 0.5 * prev)
}

fn fmt_opt_e(v: Option<f64>) -> String {
    v.map(|v| format!("{v:.4e}")).unwrap_or_else(|| "-".into())
}

fn fmt_opt_km_6(v: Option<f64>) -> String {
    v.map(|v| format!("{:.6}", v / 1e3))
        .unwrap_or_else(|| "-".into())
}

fn fmt_opt_km(v: Option<f64>) -> String {
    v.map(|v| format!("{:.4}", v / 1e3))
        .unwrap_or_else(|| "-".into())
}

fn opt_delta_km(a: Option<f64>, b: Option<f64>) -> String {
    match (a, b) {
        (Some(a), Some(b)) => format!("{:.4}", (a - b) / 1e3),
        _ => "-".into(),
    }
}
