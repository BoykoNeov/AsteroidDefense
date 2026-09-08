//! **Where is the keyhole door, really?** — the placement half of the
//! keyhole-width question, measured on more than one flown resonance.
//!
//! # The open question this exists to close
//!
//! `keyhole.rs` draws a resonant circle and puts a *width* on it:
//! `Δa'_tol / |∇a'|`, a linearisation. The flown 3:4 plan sits **20.4 km** from
//! its own circle against a **24.9 km** width — 1.64 half-widths, i.e. the map
//! says `inside = false` — and yet that plan demonstrably returns **inside
//! Earth**. So the linearised door is misplaced, or too narrow, or both, by
//! something of order 1.6. On a sample of one.
//!
//! Two different quantities hide in that sentence, and only one of them has been
//! measured since:
//!
//! - **Width** — how *wide* the door is. Settled differentially on 2026-09-06 by
//!   measuring the chained `∂ζ₂/∂ζ₁` on the flown trajectory: 29.09 km against
//!   the map's 24.92 km, conservative by **1.17×**. That number is the closed
//!   form's gain inverted, so it inherits none of the map's absolute-placement
//!   error — and it says nothing about placement.
//! - **Placement** — *where* the door's centre is. Still open. It is what the
//!   1.64 half-widths describes, and no differential measurement can reach it.
//!
//! This probe measures both, directly, by finding the **edges of the door the
//! rock actually flies through**: the two Δv at which the resonant return stops
//! hitting Earth. Each edge is a flown shot, so each has a b-plane point at
//! encounter 1, and `ResonantCircle::signed_distance` turns each into a signed
//! kilometre offset from the circle. The pair gives, per resonance:
//!
//! - **the flown door's width** — `|d_hi − d_lo|`, against `Keyhole::width`;
//! - **the flown door's centre** — `(d_hi + d_lo)/2`, against the circle's own
//!   zero. That is the placement error, in the units the panel quotes.
//!
//! A yes/no "did it return inside Earth" gives one bit per resonance. This gives
//! two numbers, which is what turns three flights into a calibration.
//!
//! # Why the aim is not solved with `required_dv`
//!
//! [`asteroid_core::DeflectionScenario::required_dv`] bisects to a target
//! perigee, and **every probe re-flies the whole campaign**: ~18 re-flights,
//! 2–4 minutes, *per resonance*. Over a survey that pays for the same
//! `b(Δv)` curve again and again — a property of the deflection, not of the
//! resonance aimed at. The `ladder` stage samples that curve once and writes it
//! out; every later aim is an interpolation of it, free.
//!
//! That is only safe because the aim exists to **bracket**, not to be right:
//! [`refine_keyhole_return`] widens by doubling and reports whether it ever
//! bracketed, so an approximate aim shows up as a longer walk or an honest
//! `bracketed = false`, never as a silently wrong floor. The ladder's own
//! accuracy is reported (`aim residual`) on every flight, so it is auditable
//! rather than assumed.
//!
//! # Stages
//!
//! ```text
//!   probe_keyhole_placement ladder [rungs]        # ~5 min, writes the b(Δv) curve
//!   probe_keyhole_placement screen [max_flights]  # ~15 s per candidate
//!   probe_keyhole_placement door h k branch dir [iters]   # ~10 min per resonance
//!   probe_keyhole_placement outgoing [h k branch] [leads]  # ~4 s per flown door
//! ```
//!
//! `outgoing` answers a different question from the rest and needs no ladder: it
//! re-flies **recorded** door centres and measures the orbit the flyby actually
//! produces, which splits the map's error into the part that is a wrong
//! *prediction* of the outgoing orbit and the part that is a wrong *condition*
//! for a return. See its own doc comment for the decision rule, which was written
//! before it ran.
//!
//! `ladder` must run first; the other two read `keyhole_ladder.tsv` from the work
//! directory (`ASTEROID_PROBE_DIR`, default `W:/temp/claude/keyhole_placement`).
//! Every stage appends to a log there, so a long survey survives being
//! interrupted.
//!
//! Requires kernels.

use anise::constants::frames::{EARTH_J2000, SSB_J2000, SUN_J2000};
use asteroid_core::{
    aim_at_resonance, along_track_unit, closest_approach, fly_keyhole_shot, refine_keyhole_return,
    BPlaneEncounter, CircleBranch, EphemerisPerturber, Epoch, ImpactorConfig, KeyholeAim,
    KeyholeAiming, KeyholeRefineTol, KeyholeShot, KeyholeShotOptions, OpikFrame, RealFieldScenario,
    Resonance, ResonantCircle, ScanOptions, StateVector, AU_M,
};
use nalgebra::{Vector2, Vector3};
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;
use std::time::Instant;

/// Census bounds — the same ones `probe_keyhole_map` draws, so this probe can
/// never consider a resonance the map does not show.
const RETURN_YEARS: std::ops::RangeInclusive<u32> = 2..=20;
const MAX_REVOLUTIONS: u32 = 24;
const B_MAX_CAPTURE_RADII: f64 = 60.0;

/// The frontend's shipping placement band, km — `sim.gd`'s `KEYHOLE_PLACEMENT_KM`.
///
/// Mirrored here **only so the log can say what it is being compared against**;
/// nothing in this probe's physics reads it. It has moved twice (100 → 500 → 800)
/// and this line was left saying 100 through both, which is exactly the kind of
/// stale number a log is worst at: it is not asserted anywhere, so nothing fails
/// when it rots, and a reader takes it for the current value.
const SHIPPING_BAND_KM: f64 = 800.0;

/// The largest door-centre offset any flown door has shown, km — the 200 d 3:4
/// door, which is what set [`SHIPPING_BAND_KM`]. Also stale for two sessions at
/// 26.8, the five-door maximum, after the lead sweep found 210.6, 467.9, 648.2
/// and 786.0 km at dialable leads.
const LARGEST_PLACEMENT_ERROR_KM: f64 = 786.0;

/// The Δv span the ladder covers **at the campaign's own 12 yr lead**, m/s. The
/// low end is below any reachable resonance; the high end is where the deflected
/// pass starts leaving the shipping 5e8 m scan gate, which the ladder reports
/// rather than assumes.
///
/// At a shorter lead the span is multiplied by `default_lead / lead` so it still
/// covers the same `b` range. **That factor is a bracket, not a law.** The
/// crossing Δv measured by `xi_sweep` on the 3:4 came out 4.30× the 12 yr value
/// at a 4.87× shorter lead and 14.9× at a 29.2× shorter one — so the 1/lead
/// scaling this factor is borrowed from is right to within 15 % at 900 days and
/// wrong by 2× at 150, partly because the crossing itself slides round the circle
/// as the lead changes and is no longer the same displacement. Scaling by it
/// over-covers, which is what a bracket is for; reading it as the physics would
/// be a claim this probe's own table contradicts.
/// Where past closest approach the `outgoing` stage reads the flyby's true
/// outgoing orbit, days. The first samples are **not** expected to be converged —
/// they are printed so the settling can be seen, and each row carries how many
/// Earth Hill radii out it is.
const SAMPLE_DAYS: [f64; 6] = [10.0, 20.0, 30.0, 60.0, 90.0, 180.0];

/// Where the revolution-mean window opens, days past closest approach.
///
/// **Calibrated to this rock's `v∞`, not derived.** What matters is the
/// *geocentric distance* at the opening — the CA+10 d sample is 350 km-equivalent
/// off every later one because at 4.4 Earth Hill radii Earth is still moving the
/// rock's heliocentric energy. At these flights' ~5 km/s, 30 days buys ~13 Hill
/// radii; a slower flyby, or a resonance reached at a lower `v∞`, would open the
/// window nearer in and quietly re-import that contamination. The stage prints
/// each sample's distance in Hill radii, and the guard test asserts the opening
/// clears [`SETTLE_HILL_RADII`], so this constant cannot rot silently — but it
/// must be re-checked, not inherited, on a different encounter.
const SETTLE_DAYS: f64 = 30.0;

/// A second opening for the same mean, days — the settling check. The
/// half-revolution shift asks whether the window is one revolution; this asks
/// whether it opened late enough. Where the two answers differ the row has not
/// settled and its number is a convention, not a measurement.
const SETTLE_LATE_DAYS: f64 = 60.0;

/// How many points the revolution-mean averages over.
const MEAN_SAMPLES: usize = 32;

/// Earth's Hill radius, metres — 0.01 AU to two figures. Only a scale for the
/// sample table's "how far out of Earth's grip is this reading" column; nothing
/// is computed from it.
const EARTH_HILL_RADIUS_M: f64 = 1.5e9;

const LADDER_DV_LO: f64 = 0.004;
const LADDER_DV_HI: f64 = 0.80;
const LADDER_RUNGS: usize = 14;

/// The ladder is a property of **one deflection epoch**: a shorter lead needs a
/// bigger `Δv` for the same `b`, so a ladder read at the wrong lead is a wrong
/// curve, silently. The lead is in the filename *and* in the file's own header,
/// and [`Ladder::load`] refuses a mismatch rather than interpolating it.
fn ladder_path(lead_days: f64) -> PathBuf {
    work_dir().join(format!("keyhole_ladder_{lead_days:.0}d.tsv"))
}

/// Pull a `lead=<days>` token out of the argument list. Returned separately from
/// the positional arguments (which are filtered) so a stage's own indices do not
/// move when a lead is given.
fn take_lead(args: &[String]) -> (Option<f64>, Vec<String>) {
    let mut lead = None;
    let mut rest = Vec::new();
    for a in args {
        match a.strip_prefix("lead=").and_then(|s| s.parse::<f64>().ok()) {
            Some(d) if d > 0.0 => lead = Some(d),
            _ => rest.push(a.clone()),
        }
    }
    (lead, rest)
}

/// Pull a `xi=<km>` token out of the argument list, same discipline as
/// [`take_lead`]. Only `spacing` reads it: crowding is a function of **where on
/// the b-plane you are asking**, and the lead sweep showed the plan's own ξ
/// ranges over tens of thousands of kilometres, so asking only at the nominal ξ
/// answers for one point of a patch the player moves around in.
fn take_xi_km(args: &[String]) -> (Option<f64>, Vec<String>) {
    let mut xi = None;
    let mut rest = Vec::new();
    for a in args {
        match a.strip_prefix("xi=").and_then(|s| s.parse::<f64>().ok()) {
            Some(x) if x.is_finite() => xi = Some(x * 1e3),
            _ => rest.push(a.clone()),
        }
    }
    (xi, rest)
}

/// Pull a `dvmax=<m/s>` token out of the argument list, same discipline as
/// [`take_lead`]. Only `xi_sweep` reads it. See [`SWEEP_DV_HI`]: its default is
/// calibrated at a 150 d lead, and a shorter lead needs a proportionally bigger
/// impulse to reach the same circle, so the ceiling has to move with the lead or
/// the scan reports the ceiling as a reachability result.
fn take_dv_max(args: &[String]) -> (Option<f64>, Vec<String>) {
    let mut dv = None;
    let mut rest = Vec::new();
    for a in args {
        match a.strip_prefix("dvmax=").and_then(|s| s.parse::<f64>().ok()) {
            Some(v) if v > 0.0 => dv = Some(v),
            _ => rest.push(a.clone()),
        }
    }
    (dv, rest)
}

/// Pull a `dv=<m/s>` token out of the argument list: the Δv the `door` stage
/// starts its refinement from, overriding the ladder's.
///
/// **The ladder's aim is not usable below ~150 days, and that is a measurement,
/// not a suspicion.** The ladder maps Δv to `b`, and the aim asks for the `b` of
/// the circle's point at the *nominal* ξ. On the 3:4 that point sits at
/// b = 153 424 km against a circle whose largest possible `b` is 153 577 km — it
/// is essentially the circle's outermost point, reachable only near ξ = 0. At a
/// short lead the reachable curve gets to that same `b` far out in ξ, so the aim
/// is right in `b` and hundreds of thousands of kilometres from the door. Flown
/// at 125, 100, 75 and 50 d it produced `bracketed = false` four times, ending
/// 214 729 to 320 646 km outside the circle.
///
/// So a short-lead door is aimed from `xi_sweep`'s **crossing** Δv, which is
/// where the curve actually meets the circle, and this is how that gets in.
fn take_dv_aim(args: &[String]) -> (Option<f64>, Vec<String>) {
    let mut dv = None;
    let mut rest = Vec::new();
    for a in args {
        match a.strip_prefix("dv=").and_then(|s| s.parse::<f64>().ok()) {
            Some(v) if v.is_finite() && v > 0.0 => dv = Some(v),
            _ => rest.push(a.clone()),
        }
    }
    (dv, rest)
}

/// Pull a `rungs=<n>` token out of the argument list. Raising [`SWEEP_DV_HI`]
/// without raising this coarsens the *geometric* spacing of the scan, and the
/// scan's whole job is to catch a sign change between adjacent rungs — so the
/// two knobs belong together and are documented as a pair.
fn take_rungs(args: &[String]) -> (Option<usize>, Vec<String>) {
    let mut n = None;
    let mut rest = Vec::new();
    for a in args {
        match a
            .strip_prefix("rungs=")
            .and_then(|s| s.parse::<usize>().ok())
        {
            Some(v) if v >= 3 => n = Some(v),
            _ => rest.push(a.clone()),
        }
    }
    (n, rest)
}

fn work_dir() -> PathBuf {
    std::env::var("ASTEROID_PROBE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("W:/temp/claude/keyhole_placement"))
}

/// Append a line to the stage log *and* print it, so an interrupted run leaves
/// everything it had measured on disk.
fn log(line: &str) {
    println!("{line}");
    let dir = work_dir();
    let _ = std::fs::create_dir_all(&dir);
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(dir.join("placement_log.txt"))
    {
        let _ = writeln!(f, "{line}");
    }
}

/// One rung: an impulse and the encounter-1 geometry it produces.
#[derive(Clone, Copy)]
struct Rung {
    /// Signed: negative is the retrograde direction. The sign is what selects the
    /// ζ side, so it must survive into the table.
    dv_signed: f64,
    b_m: f64,
    /// Written to the TSV and re-read into the struct; kept as a column because
    /// the ξ drift with Δv is the reason an aim holds the *nominal* ξ.
    #[allow(dead_code)]
    xi_m: f64,
    zeta_m: f64,
}

struct Ladder {
    rungs: Vec<Rung>,
}

impl Ladder {
    /// Read a ladder, **refusing one flown at a different lead**. The header
    /// carries `# lead_days = X`; a file without it is from before this probe
    /// knew leads existed and is refused too, because "assume it is the default"
    /// is exactly the silent wrong-curve read this check exists to stop.
    fn load(path: &std::path::Path, lead_days: f64) -> Option<Self> {
        let text = std::fs::read_to_string(path).ok()?;
        let header = text.lines().next()?;
        let stated = header
            .strip_prefix("# lead_days = ")
            .and_then(|s| s.trim().parse::<f64>().ok());
        match stated {
            Some(d) if (d - lead_days).abs() <= 0.5 => {}
            Some(d) => {
                eprintln!(
                    "{} was flown at lead {d:.0} d, not {lead_days:.0} d — refusing to                      interpolate the wrong curve",
                    path.display()
                );
                return None;
            }
            None => {
                eprintln!(
                    "{} has no lead header — re-run the `ladder` stage to write one",
                    path.display()
                );
                return None;
            }
        }
        let mut rungs = Vec::new();
        for line in text.lines().skip(2) {
            let f: Vec<f64> = line
                .split('\t')
                .filter_map(|s| s.trim().parse::<f64>().ok())
                .collect();
            if f.len() >= 4 {
                rungs.push(Rung {
                    dv_signed: f[0],
                    b_m: f[1],
                    xi_m: f[2],
                    zeta_m: f[3],
                });
            }
        }
        (!rungs.is_empty()).then_some(Self { rungs })
    }

    /// The Δv (magnitude) whose flyby has impact parameter `b`, on the ζ side of
    /// `sign`. Log-log interpolation between the two bracketing rungs: `b` grows
    /// very nearly linearly with Δv once the nudge dominates, so this is a
    /// near-exact inversion in the middle of the range and a mild extrapolation
    /// at the ends. `None` when `b` is outside the sampled range — an
    /// extrapolation past the ends would be a guess with no error bar.
    fn dv_for_b(&self, b: f64, sign: f64) -> Option<f64> {
        let mut side: Vec<&Rung> = self
            .rungs
            .iter()
            .filter(|r| r.dv_signed.signum() == sign && r.b_m > 0.0)
            .collect();
        side.sort_by(|a, b| a.b_m.partial_cmp(&b.b_m).expect("finite"));
        if side.len() < 2 || b < side[0].b_m || b > side[side.len() - 1].b_m {
            return None;
        }
        let i = side.partition_point(|r| r.b_m < b).max(1);
        let (lo, hi) = (side[i - 1], side[i]);
        let t = (b.ln() - lo.b_m.ln()) / (hi.b_m.ln() - lo.b_m.ln());
        Some(
            (lo.dv_signed.abs().ln() + t * (hi.dv_signed.abs().ln() - lo.dv_signed.abs().ln()))
                .exp(),
        )
    }

    /// `dζ/dΔv` at the rung pair straddling `dv` on the `sign` side, metres per
    /// m/s — the slope that converts a Δv span into a b-plane span.
    fn dzeta_ddv(&self, dv: f64, sign: f64) -> Option<f64> {
        let mut side: Vec<&Rung> = self
            .rungs
            .iter()
            .filter(|r| r.dv_signed.signum() == sign)
            .collect();
        side.sort_by(|a, b| {
            a.dv_signed
                .abs()
                .partial_cmp(&b.dv_signed.abs())
                .expect("finite")
        });
        if side.len() < 2 {
            return None;
        }
        let i = side
            .partition_point(|r| r.dv_signed.abs() < dv)
            .clamp(1, side.len() - 1);
        let (lo, hi) = (side[i - 1], side[i]);
        Some((hi.zeta_m - lo.zeta_m) / (hi.dv_signed.abs() - lo.dv_signed.abs()))
    }
}

/// Everything the stages share: the scenario, its nominal encounter, the Öpik
/// frame and the two impulse directions.
struct Setup {
    scenario: RealFieldScenario,
    nominal: BPlaneEncounter,
    frame: OpikFrame,
    /// When the impulse is applied. **Not** necessarily `scenario.epoch0()` any
    /// more: the frontend's lead slider is a live knob (30..900 d), so the lead
    /// is a stage argument and every number a run reports belongs to one.
    deflection_epoch: asteroid_core::Epoch,
    /// That epoch as a lead before impact, days — carried so it lands in the log
    /// line and the ladder filename rather than being implied by the run.
    lead_days: f64,
    /// The campaign's own lead (`scenario.epoch0()`), days. Every Δv range in
    /// this probe was tuned at it, so it is the reference a shorter lead's range
    /// is scaled against.
    default_lead_days: f64,
    prograde: Vector3<f64>,
    /// The ξ every aim holds: the nominal encounter's own, which a small
    /// along-track nudge barely moves. Same convention as `probe_keyhole_return`,
    /// so the numbers here are commensurable with the published 3:4 ones.
    xi: f64,
}

/// Build the scenario and the nominal encounter's frame, with the impulse epoch
/// set by `lead_days` (default: the campaign's own `epoch0`, 12 yr).
///
/// The scenario, the nominal encounter and the Öpik frame are all properties of
/// the **undeflected** rock, so none of them depends on the lead — one build
/// serves a whole sweep.
fn setup(lead_days: Option<f64>) -> Setup {
    let t = Instant::now();
    let scenario = RealFieldScenario::build(&ImpactorConfig::default()).unwrap_or_else(|e| {
        eprintln!("build failed: {e}");
        std::process::exit(1);
    });
    log(&format!("build: {:.1} s", t.elapsed().as_secs_f64()));
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
    let impact = scenario.impact_epoch();
    let default_lead_days =
        (impact.tdb_seconds_past_j2000() - scenario.epoch0().tdb_seconds_past_j2000()) / 86_400.0;
    let deflection_epoch = match lead_days {
        Some(d) => impact.shifted_by_seconds(-d * 86_400.0),
        None => scenario.epoch0(),
    };
    let lead_days = lead_days.unwrap_or_else(|| {
        (impact.tdb_seconds_past_j2000() - deflection_epoch.tdb_seconds_past_j2000()) / 86_400.0
    });
    let seed = ds.nominal().state_at(deflection_epoch).expect("seed");
    let prograde = along_track_unit(seed).expect("along-track");
    let xi = frame.project(&nominal.b_vector).x;
    drop(ds);
    log(&format!("lead: {lead_days:.1} d"));
    Setup {
        scenario,
        nominal,
        frame,
        deflection_epoch,
        lead_days,
        default_lead_days,
        prograde,
        xi,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let stage = args.first().map(String::as_str).unwrap_or("screen");
    log(&format!(
        "\n=== probe_keyhole_placement {stage} === work dir {}",
        work_dir().display()
    ));
    match stage {
        "ladder" => stage_ladder(&args),
        "screen" => stage_screen(&args),
        "door" => stage_door(&args),
        "spacing" => stage_spacing(&args),
        "xi_sweep" => stage_xi_sweep(&args),
        "frame" => stage_frame(&args),
        "crowding" => stage_crowding(&args),
        "outgoing" => stage_outgoing(&args),
        other => {
            eprintln!(
                "unknown stage {other:?}; expected ladder | screen | door | spacing | xi_sweep | frame | crowding | outgoing"
            );
            std::process::exit(2);
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 1 — the b(Δv) curve, sampled once
// ---------------------------------------------------------------------------

fn stage_ladder(args: &[String]) {
    let (lead, args) = take_lead(args);
    let rungs: usize = args
        .get(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(LADDER_RUNGS);
    let s = setup(lead);
    let ds = s.scenario.deflection().expect("deflection");
    let _ = std::fs::create_dir_all(work_dir());
    let path = ladder_path(s.lead_days);
    let mut out =
        format!("# lead_days = {:.4}\n", s.lead_days) + "dv_signed_m_s\tb_m\txi_m\tzeta_m\n";

    log(&format!(
        "nominal: ξ {:.1} km, ζ {:.1} km, b {:.1} km, capture {:.1} km",
        s.xi / 1e3,
        s.frame.project(&s.nominal.b_vector).y / 1e3,
        s.nominal.impact_parameter / 1e3,
        s.nominal.capture_radius / 1e3
    ));
    log("   dv (m/s)      b (km)     xi (km)    zeta (km)   perigee (km)");
    let t = Instant::now();
    let mut flights = 0usize;
    let span = s.default_lead_days / s.lead_days;
    let (dv_lo, dv_hi) = (LADDER_DV_LO * span, LADDER_DV_HI * span);
    if span != 1.0 {
        log(&format!(
            "lead {:.0} d is {span:.2}x shorter than the campaign's {:.0} d, so the ladder \
             spans {dv_lo:.4} .. {dv_hi:.4} m/s (see LADDER_DV_LO: a bracket, not a law)",
            s.lead_days, s.default_lead_days
        ));
    }
    for sign in [-1.0f64, 1.0] {
        for i in 0..rungs {
            let f = i as f64 / (rungs - 1) as f64;
            let dv = dv_lo * (dv_hi / dv_lo).powf(f);
            let delta = s.prograde * (sign * dv);
            flights += 1;
            match ds.evaluate(s.deflection_epoch, delta) {
                Ok(Some(enc)) => {
                    let p = s.frame.project(&enc.b_vector);
                    let _ = writeln!(
                        out,
                        "{:.10}\t{:.6}\t{:.6}\t{:.6}",
                        sign * dv,
                        enc.impact_parameter,
                        p.x,
                        p.y
                    );
                    log(&format!(
                        "{:11.6} {:11.1} {:11.1} {:12.1} {:14.1}",
                        sign * dv,
                        enc.impact_parameter / 1e3,
                        p.x / 1e3,
                        p.y / 1e3,
                        enc.perigee / 1e3
                    ));
                }
                // Past the scan gate the pass is a clean miss with no reduction.
                // That is the ladder's own top end, reported rather than assumed.
                Ok(None) => log(&format!(
                    "{:11.6}   -- left the 5e8 m scan gate; the ladder ends here --",
                    sign * dv
                )),
                Err(e) => log(&format!("{:11.6}   -- {e} --", sign * dv)),
            }
        }
    }
    std::fs::write(&path, &out).expect("write ladder");
    log(&format!(
        "\n{flights} re-flights in {:.0} s → {}",
        t.elapsed().as_secs_f64(),
        path.display()
    ));

    // The consistency check that validates the whole method before a single
    // keyhole is flown: the measured Δv→ζ slope must turn the 3:4's independently
    // measured door in Δv (~±2e-5 m/s, `probe_keyhole_floor`) into the door width
    // measured differentially (29.09 km, `probe_keyhole_probability`). Two
    // unrelated measurements meeting is the evidence; agreement is not assumed.
    // **Only at the campaign's own lead.** Both numbers this checks against were
    // measured there: the +/-2e-5 m/s door is a span in *dv*, and dv buys less
    // b-plane at a shorter lead, so at 450 d it reads 0.16x and at 150 d 0.06x —
    // which is the leverage changing, not a check failing. Printing it anyway
    // would put an apparent 6x disagreement in the log of every short-lead run.
    if (s.lead_days - s.default_lead_days).abs() > 0.5 {
        log(&format!(
            "
(the 3:4 cross-check is only meaningful at the campaign's {:.0} d lead —              its +/-2e-5 m/s door is a span in dv, and dv buys less b-plane here)",
            s.default_lead_days
        ));
    } else if let Some(l) = Ladder::load(&path, s.lead_days) {
        if let Some(slope) = l.dzeta_ddv(0.2165, -1.0) {
            let span_km = 2.0 * 2.0e-5 * slope.abs() / 1e3;
            log(&format!(
                "\ncross-check: dζ/dΔv near the 3:4 floor = {:.4e} km per m/s\n  \
                 × the ±2e-5 m/s door `probe_keyhole_floor` measured = {span_km:.1} km,\n  \
                 against the 29.09 km `probe_keyhole_probability` measured differentially \
                 ({:.2}×).",
                slope.abs() / 1e3,
                span_km / 29.09
            ));
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 2 — which resonances are worth an expensive refine
// ---------------------------------------------------------------------------

/// One candidate aim, before anything has been flown.
struct Candidate {
    circle: ResonantCircle,
    branch: CircleBranch,
    aim: KeyholeAim,
    /// Half the linearised keyhole width **at the aim point** — not at the
    /// circle's far point, which is a different door and differs by 100× on some
    /// circles.
    half_width_m: f64,
    dv: f64,
    sign: f64,
}

fn candidates(s: &Setup, ladder: &Ladder) -> Vec<Candidate> {
    let circles = s.frame.resonant_circles(
        RETURN_YEARS,
        MAX_REVOLUTIONS,
        B_MAX_CAPTURE_RADII * s.nominal.capture_radius,
    );
    let mut out = Vec::new();
    for c in &circles {
        for branch in [CircleBranch::Minus, CircleBranch::Plus] {
            // `points_at_xi` refuses a ξ wider than the circle — a geometric
            // fact, and the free half of the pre-filter.
            let Ok(aim) = aim_at_resonance(&s.frame, c.resonance, s.xi, branch) else {
                continue;
            };
            // A perigee inside Earth is an impact at encounter 1, not a keyhole:
            // no deflection solver can aim at it.
            if aim.perigee_m <= s.nominal.earth_radius {
                continue;
            }
            let sign = if aim.target.y < 0.0 { -1.0 } else { 1.0 };
            let Some(dv) = ladder.dv_for_b(aim.impact_parameter_m, sign) else {
                continue;
            };
            let kh = s.frame.keyhole_at(c, aim.target);
            if !kh.width.is_finite() {
                continue;
            }
            out.push(Candidate {
                circle: *c,
                branch,
                aim,
                half_width_m: 0.5 * kh.width,
                dv,
                sign,
            });
        }
    }
    // Shortest return first. The one prediction worth testing on the way past:
    // the 3:4 (h = 3) floors 4 006 km out and the 7:9 (h = 7) at 46 608 km, so
    // the spatial offset looks like it grows with the years spent on the
    // resonant orbit — and short returns are also the cheap ones to fly.
    out.sort_by_key(|c| (c.circle.resonance.h, c.circle.resonance.k));
    out
}

// ---------------------------------------------------------------------------
// Stage 4 — how close together the drawn circles sit (no flights)
// ---------------------------------------------------------------------------
//
// The frontend alerts when a b-plane point is within some band of a drawn
// resonant circle. If two circles sit closer together than that band, the
// alert is on permanently and says nothing. This is the closed-form check on
// that, and it costs nothing: `aim_at_resonance` is geometry, not propagation.

fn stage_spacing(args: &[String]) {
    // The frontend draws only what `KEYHOLE_MAX_YEARS` admits (7), so the census
    // this check runs on has to be the drawn one, not the wider survey census.
    let (lead, args) = take_lead(args);
    let (xi_override, args) = take_xi_km(&args);
    let max_years: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(7);
    let s = setup(lead);
    let xi = xi_override.unwrap_or(s.xi);
    let circles = s.frame.resonant_circles(
        2..=max_years.max(2),
        MAX_REVOLUTIONS,
        B_MAX_CAPTURE_RADII * s.nominal.capture_radius,
    );
    let mut bs: Vec<(f64, String)> = Vec::new();
    for c in &circles {
        for branch in [CircleBranch::Minus, CircleBranch::Plus] {
            let Ok(aim) = aim_at_resonance(&s.frame, c.resonance, xi, branch) else {
                continue;
            };
            if aim.perigee_m <= s.nominal.earth_radius {
                continue;
            }
            bs.push((
                aim.impact_parameter_m,
                format!("{}:{} {:?}", c.resonance.h, c.resonance.k, branch),
            ));
        }
    }
    bs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    log(&format!(
        "\n{} reachable circles at ξ = {:.0} km{}. Nearest-neighbour gaps in b:\n",
        bs.len(),
        xi / 1e3,
        if xi_override.is_some() {
            " (asked)"
        } else {
            " (the nominal encounter's own)"
        }
    ));
    let mut worst = (f64::INFINITY, String::new());
    for w in bs.windows(2) {
        let gap = w[1].0 - w[0].0;
        if gap < worst.0 {
            worst = (gap, format!("{} .. {}", w[0].1, w[1].1));
        }
        if gap < 5.0e5 {
            log(&format!(
                "  {:>12.1} km gap   {:>12} b {:10.0} km  ..  {:>12} b {:10.0} km",
                gap / 1e3,
                w[0].1,
                w[0].0 / 1e3,
                w[1].1,
                w[1].0 / 1e3
            ));
        }
    }
    log(&format!(
        "\ntightest pair anywhere in the census: {:.1} km  ({})",
        worst.0 / 1e3,
        worst.1
    ));
}

fn stage_screen(args: &[String]) {
    let (lead, args) = take_lead(args);
    let max_flights: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(14);
    let s = setup(lead);
    let Some(ladder) = Ladder::load(&ladder_path(s.lead_days), s.lead_days) else {
        eprintln!(
            "no usable ladder at {} — run `ladder lead={:.0}` first",
            ladder_path(s.lead_days).display(),
            s.lead_days
        );
        std::process::exit(3);
    };
    let ds = s.scenario.deflection().expect("deflection");
    let cands = candidates(&s, &ladder);
    log(&format!(
        "\n{} reachable aims at ξ = {:.0} km (of {} circles in the census)\n",
        cands.len(),
        s.xi / 1e3,
        s.frame
            .resonant_circles(
                RETURN_YEARS,
                MAX_REVOLUTIONS,
                B_MAX_CAPTURE_RADII * s.nominal.capture_radius
            )
            .len()
    ));
    log(
        "  h:k  br      a' (AU)     b (km)  half-w (km)   dv (m/s) | flown: xi2 (km)  \
         zeta2 (km)  return (km)  hit?",
    );

    let aiming = KeyholeAiming {
        frame: &s.frame,
        earth_radius_m: s.nominal.earth_radius,
        deflection_epoch: s.deflection_epoch,
        direction: s.prograde,
    };
    let t = Instant::now();
    for c in cands.iter().take(max_flights) {
        let head = format!(
            "{:>3}:{:<3} {:?}  {:9.6} {:10.0} {:12.3} {:10.6}",
            c.circle.resonance.h,
            c.circle.resonance.k,
            c.branch,
            c.circle.a_prime / AU_M,
            c.aim.impact_parameter_m / 1e3,
            c.half_width_m * 2.0 / 1e3,
            c.dv
        );
        let aiming = KeyholeAiming {
            direction: s.prograde * c.sign,
            ..aiming
        };
        let opts = KeyholeShotOptions::default().widened_for_aim(c.aim.impact_parameter_m);
        match fly_keyhole_shot(&s.scenario, &ds, aiming, c.dv, c.circle.resonance, &opts) {
            Err(e) => log(&format!("{head} | -- {e} --")),
            Ok(shot) => {
                // How well the ladder's interpolation actually aimed, in the
                // coordinate that matters. Reported every flight so the shortcut
                // is auditable instead of trusted.
                let residual = shot.encounter.impact_parameter - c.aim.impact_parameter_m;
                match shot.flown_return.as_ref() {
                    None => log(&format!(
                        "{head} | no return in the gate (aim residual {:.0} km of b)",
                        residual / 1e3
                    )),
                    Some(r) => log(&format!(
                        "{head} | {:12.0} {:11.0} {:12.0}  {}   (aim residual {:.0} km of b)",
                        r.xi_m.unwrap_or(f64::NAN) / 1e3,
                        r.zeta_m.unwrap_or(f64::NAN) / 1e3,
                        r.distance_m / 1e3,
                        match r.encounter.as_ref() {
                            Some(e) if e.is_hit() => "HIT",
                            Some(_) => "miss",
                            None => "no-red",
                        },
                        residual / 1e3
                    )),
                }
            }
        }
    }
    log(&format!(
        "\nscreen: {} flights in {:.0} s. The column to read is xi2 — the spatial \
         offset the refinement cannot remove, so a candidate whose |ξ₂| is already \
         far outside Earth cannot become an impact keyhole no matter how the timing \
         is spent.",
        max_flights.min(cands.len()),
        t.elapsed().as_secs_f64()
    ));
}

// ---------------------------------------------------------------------------
// Stage 3 — the flown door: refine to the floor, then find both edges
// ---------------------------------------------------------------------------

fn stage_door(args: &[String]) {
    let (lead, args) = take_lead(args);
    let (dv_override, args) = take_dv_aim(&args);
    let h: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(3);
    let k: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4);
    let branch = match args.get(3).map(|s| s.to_lowercase()) {
        Some(s) if s.starts_with('p') => CircleBranch::Plus,
        _ => CircleBranch::Minus,
    };
    let iterations: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(20);
    let resonance = Resonance { h, k };

    let s = setup(lead);
    let Some(ladder) = Ladder::load(&ladder_path(s.lead_days), s.lead_days) else {
        eprintln!(
            "no usable ladder at {} — run `ladder lead={:.0}` first",
            ladder_path(s.lead_days).display(),
            s.lead_days
        );
        std::process::exit(3);
    };
    let ds = s.scenario.deflection().expect("deflection");
    let aim = aim_at_resonance(&s.frame, resonance, s.xi, branch).unwrap_or_else(|e| {
        eprintln!("cannot aim at {resonance}: {e}");
        std::process::exit(1);
    });
    let sign = if aim.target.y < 0.0 { -1.0 } else { 1.0 };
    let ladder_dv = ladder.dv_for_b(aim.impact_parameter_m, sign);
    let aim_dv = match (dv_override, ladder_dv) {
        (Some(dv), _) => dv,
        (None, Some(dv)) => dv,
        (None, None) => {
            eprintln!(
                "{resonance} wants b = {:.0} km, outside the ladder's range, and no                  dv= aim was given",
                aim.impact_parameter_m / 1e3
            );
            std::process::exit(1);
        }
    };
    let circle = aim.circle;
    let kh = s.frame.keyhole_at(&circle, aim.target);
    log(&format!(
        "{resonance} {branch:?}: a' {:.6} AU, aim (ξ {:.0}, ζ {:.0}) km, b {:.0} km, \
         perigee {:.0} km\n  linearised door at the aim point: {:.3} km wide \
         (|∇a'| {:.3e}); aim Δv {:.10} m/s from {} ({} nudge)",
        circle.a_prime / AU_M,
        aim.target.x / 1e3,
        aim.target.y / 1e3,
        aim.impact_parameter_m / 1e3,
        aim.perigee_m / 1e3,
        kh.width / 1e3,
        kh.gradient.norm(),
        aim_dv,
        match (dv_override, ladder_dv) {
            // Said out loud on every run, because which of these two aimed the
            // shot is the difference between a door and a `bracketed = false`.
            (Some(_), Some(l)) => format!("dv= (the ladder would have said {l:.6})"),
            (Some(_), None) => "dv= (the ladder could not reach this b)".to_string(),
            (None, _) => "the ladder".to_string(),
        },
        if sign < 0.0 { "retrograde" } else { "prograde" }
    ));

    let aiming = KeyholeAiming {
        frame: &s.frame,
        earth_radius_m: s.nominal.earth_radius,
        deflection_epoch: s.deflection_epoch,
        direction: s.prograde * sign,
    };
    let tol = KeyholeRefineTol {
        max_iterations: iterations,
        ..Default::default()
    };
    let t = Instant::now();
    let solution = match refine_keyhole_return(
        &s.scenario,
        &ds,
        aiming,
        resonance,
        aim,
        aim_dv,
        &KeyholeShotOptions::default(),
        tol,
    ) {
        Ok(s) => s,
        Err(e) => {
            log(&format!("refine failed: {e}"));
            std::process::exit(1);
        }
    };
    log(&format!(
        "\nrefine: {} flights in {:.0} s, floor Δv {:.10} m/s, bracketed = {}",
        solution.flights,
        t.elapsed().as_secs_f64(),
        solution.best.dv_m_s,
        solution.bracketed
    ));
    report_shot("floor", &solution.best, &circle);

    // A search that never bracketed is sitting on its own interval wall and
    // reports a *shrinking* window while doing it. Every number below would be a
    // number from the wall, so say so and stop.
    if !solution.bracketed {
        log(
            "!! THE SEARCH NEVER BRACKETED. This is not a floor and there is no door to \
             measure here: raise max_widenings or fix the aim. NOT MEASURED is the \
             result, not \"not an impact keyhole\".",
        );
        return;
    }
    if !solution.is_impact_return() {
        log(&format!(
            "the {resonance} return floors {:.0} km out — a resonant return, not an impact \
             keyhole, so it has no door to measure. (|ζ₂|/|ξ₂| = {:.3}: {} the timing is \
             spent, so this is the orbits' own offset.)",
            solution.best.return_distance_m() / 1e3,
            solution
                .best
                .flown_return
                .as_ref()
                .and_then(|r| r.timing_share())
                .unwrap_or(f64::NAN),
            if solution
                .best
                .flown_return
                .as_ref()
                .and_then(|r| r.timing_share())
                .is_some_and(|t| t < 1.0)
            {
                ""
            } else {
                "NO —"
            }
        ));
        return;
    }

    // --- the door's two edges -------------------------------------------------
    //
    // The return distance is V-shaped in Δv, so "hits Earth" is one interval
    // around the floor. Walk outward by doubling until the return stops hitting,
    // then bisect the crossing. Each probe is one flight.
    let fly = |dv: f64| -> Option<KeyholeShot> {
        let opts = KeyholeShotOptions::default().widened_for_aim(solution.aim.impact_parameter_m);
        fly_keyhole_shot(&s.scenario, &ds, aiming, dv, resonance, &opts).ok()
    };
    let hits = |shot: &Option<KeyholeShot>| -> bool {
        shot.as_ref()
            .and_then(|s| s.flown_return.as_ref())
            .and_then(|r| r.encounter.as_ref())
            .is_some_and(|e| e.is_hit())
    };
    // Seed the walk from the Δv span the linearised door implies, through the
    // measured Δv→ζ slope. Starting from the *predicted* width is deliberate: if
    // the prediction is right the walk is one step, and if it is wrong the number
    // of doublings is itself a reading.
    let slope = ladder
        .dzeta_ddv(solution.best.dv_m_s, sign)
        .unwrap_or(7.0e8);
    let seed = (kh.width / slope.abs()).max(1e-9);
    let floor_dv = solution.best.dv_m_s;
    let mut edges: Vec<(f64, KeyholeShot)> = Vec::new();
    let mut probes = 0usize;
    for out in [-1.0f64, 1.0] {
        let mut step = seed;
        let mut inside = floor_dv;
        let mut outside = None;
        for _ in 0..12 {
            let dv = floor_dv + out * step;
            if dv <= 0.0 {
                break;
            }
            let shot = fly(dv);
            probes += 1;
            if hits(&shot) {
                inside = dv;
                step *= 2.0;
            } else {
                outside = Some(dv);
                break;
            }
        }
        let Some(mut out_dv) = outside else {
            log(&format!(
                "  edge {}: still hitting {:.1e} m/s from the floor after 12 doublings — \
                 the door is wider than this walk and is NOT measured on this side",
                if out < 0.0 { "lo" } else { "hi" },
                (inside - floor_dv).abs()
            ));
            continue;
        };
        // Bisect the crossing to 1 % of the bracket. The edge reported is the
        // last shot that still *hit*, so the door is quoted conservatively (the
        // true edge is between it and the first miss).
        let mut in_dv = inside;
        let mut last = fly(in_dv);
        probes += 1;
        for _ in 0..7 {
            let mid = 0.5 * (in_dv + out_dv);
            let shot = fly(mid);
            probes += 1;
            if hits(&shot) {
                in_dv = mid;
                last = shot;
            } else {
                out_dv = mid;
            }
        }
        if let Some(shot) = last {
            log(&format!(
                "  edge {}: Δv {:.10} m/s (bracket {:.2e} m/s)",
                if out < 0.0 { "lo" } else { "hi" },
                in_dv,
                (out_dv - in_dv).abs()
            ));
            report_shot("  edge", &shot, &circle);
            edges.push((in_dv, shot));
        }
    }

    log(&format!("\ndoor: {probes} extra flights"));
    if edges.len() != 2 {
        log(
            "only one edge was found, so the door has a side but no width. Reported as \
             partial rather than halved-and-doubled.",
        );
        return;
    }
    let d_lo = circle.signed_distance(edges[0].1.point);
    let d_hi = circle.signed_distance(edges[1].1.point);
    let d_floor = circle.signed_distance(solution.best.point);
    let flown_width = (d_hi - d_lo).abs();
    let flown_centre = 0.5 * (d_hi + d_lo);
    log(&format!(
        "\n=== {resonance} {branch:?}, FLOWN DOOR vs LINEARISED DOOR ===\n\
         edges at signed distance from the circle: {:.3} km and {:.3} km\n\
         flown door WIDTH  : {:.3} km   vs linearised {:.3} km   → linearised is \
         {:.3}x the flown\n\
         flown door CENTRE : {:+.3} km from the circle (the placement error), \
         = {:.3} linearised half-widths\n\
         the floor shot itself sits {:+.3} km from the circle = {:.3} half-widths, and it \
         HITS EARTH — which is the whole point: the map calls that {}",
        d_lo / 1e3,
        d_hi / 1e3,
        flown_width / 1e3,
        kh.width / 1e3,
        kh.width / flown_width,
        flown_centre / 1e3,
        flown_centre / (0.5 * kh.width),
        d_floor / 1e3,
        d_floor.abs() / (0.5 * kh.width),
        if d_floor.abs() <= 0.5 * kh.width {
            "inside its door"
        } else {
            "OUTSIDE its door"
        }
    ));
}

// ---------------------------------------------------------------------------
// Stage 5 — the same circle at several leads: does the crossing move along it?
// ---------------------------------------------------------------------------
//
// The five flown doors each sit at one point of one circle, so the placement
// error is characterised per circle and never *along* one. The knob that moves
// along a circle is not a sideways impulse — it is the **lead time**, which the
// frontend already exposes (`sim.gd`: 30..900 d, on top of Δv 0.1..300 m/s). So
// the set of b-plane points a player can occupy is a 2-D patch, every resonant
// circle is crossed by a whole family of `(lead, Δv)` pairs, and asking the same
// circle at several leads separates *position on the circle* from *which
// resonance it was* — the confound the five-door batch could not remove.
//
// This stage is the **gate on that campaign, not the campaign**. It flies no
// returns and measures no doors: it finds where the along-track curve crosses one
// chosen circle at each lead, and reports how far apart in ξ those crossings are.
// Three outcomes, decided before running:
//
//   * ξ spread ≫ 27 km (the largest placement error the five doors showed) — the
//     sweep separates the two, and refine-and-bisect at each lead is worth its
//     ~12 minutes per door.
//   * ξ spread of order the door widths themselves (0.08–27 km) — the patch is
//     effectively one point per circle as far as this question goes, which
//     *confirms* the five-point basis of `KEYHOLE_PLACEMENT_KM` rather than
//     extending it, and the expensive campaign should not be run.
//   * the circle unreachable at some lead — a reachability finding in its own
//     right, since it says which resonances the panel can warn about when.

/// Δv range the crossing search sweeps, m/s.
///
/// The low end has to be under the **near** crossing at *every* lead, not just at
/// the short ones: a vertical line cuts a resonant circle twice, and at the 12 yr
/// lead `Δv = 0.01` already puts ζ past the near crossing, so a scan starting
/// there silently reports whichever branch it happened to open inside. That is
/// exactly the wrong-branch comparison the first run of this stage produced —
/// 7 859 km against 153 448 km of `b`, and doors 240× apart, presented as the same
/// circle at four leads. The high end is well past what the 1/lead law needs to
/// reach the far branch at 150 days, and still inside `sim.gd`'s `DV_MAX` of 300.
///
/// **The default high end is calibrated at 150 days and is NOT enough below it** —
/// the sentence above says "at 150 days" and means it. `Δv` for a given `b` grows
/// roughly as `1/lead`, so the 3:4's own aim needs 19.0 m/s at 75 d and 37.7 m/s
/// at 50 d, and a 30 m/s ceiling stops the scan before the circle is reached. A
/// scan that stops short reports `NOT REACHED`, which reads like a reachability
/// *finding* and is really the ceiling talking. Hence `dvmax=` and `rungs=`: below
/// 150 d, raise the ceiling to the app's own `DV_MAX` region and add rungs to keep
/// the geometric spacing, or the sweep answers a question about this constant
/// instead of about the rock.
const SWEEP_DV_LO: f64 = 0.001;
const SWEEP_DV_HI: f64 = 30.0;
const SWEEP_RUNGS: usize = 26;
/// Bisections of the bracketing rung pair. The crossing is reported with its own
/// residual `|d|`, so this is a cost knob and not an accuracy claim.
const SWEEP_BISECTIONS: usize = 10;

/// One lead's crossing of the chosen circle.
struct Crossing {
    lead_days: f64,
    dv: f64,
    point: nalgebra::Vector2<f64>,
    b_m: f64,
    /// Signed distance of the reported point from the circle, metres — the
    /// residual of the search, not a physical offset.
    residual_m: f64,
    /// The linearised keyhole width *at this crossing*, metres. It is a function
    /// of position on the circle, so a sweep that moves ξ also moves this.
    width_m: f64,
}

fn stage_xi_sweep(args: &[String]) {
    let (_, args) = take_lead(args);
    let (dv_max, args) = take_dv_max(&args);
    let (rungs, args) = take_rungs(&args);
    let dv_hi = dv_max.unwrap_or(SWEEP_DV_HI);
    let rungs = rungs.unwrap_or(SWEEP_RUNGS);
    let h: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(3);
    let k: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4);
    let branch = match args.get(3).map(|s| s.to_lowercase()) {
        Some(s) if s.starts_with('p') => CircleBranch::Plus,
        _ => CircleBranch::Minus,
    };
    let resonance = Resonance { h, k };
    let leads: Vec<f64> = {
        let given: Vec<f64> = args[4.min(args.len())..]
            .iter()
            .filter_map(|s| s.parse::<f64>().ok())
            .filter(|d| *d > 0.0)
            .collect();
        if given.is_empty() {
            // The campaign's own lead, then the frontend's ceiling and two points
            // below it — the region the shipping band is actually consumed in.
            vec![4383.0, 900.0, 450.0, 150.0]
        } else {
            given
        }
    };

    // The scenario, the nominal encounter and its Öpik frame are properties of the
    // undeflected rock, so one build serves every lead and the circle below is the
    // *same* circle at all of them — which is what makes the comparison a
    // comparison.
    let s = setup(None);
    let ds = s.scenario.deflection().expect("deflection");
    let aim = aim_at_resonance(&s.frame, resonance, s.xi, branch).unwrap_or_else(|e| {
        eprintln!("cannot aim at {resonance}: {e}");
        std::process::exit(1);
    });
    let circle = aim.circle;
    let sign = if aim.target.y < 0.0 { -1.0 } else { 1.0 };
    log(&format!(
        "\n{resonance} {branch:?}: a' {:.6} AU, circle centre ({:.0}, {:.0}) km radius \
         {:.0} km\n  reference crossing at the nominal ξ: (ξ {:.1}, ζ {:.1}) km, b {:.0} km, \
         linearised door {:.3} km\n  sweeping the lead over {:?} days, {} nudge",
        circle.a_prime / AU_M,
        0.0_f64,
        circle.center_zeta / 1e3,
        circle.radius / 1e3,
        aim.target.x / 1e3,
        aim.target.y / 1e3,
        aim.impact_parameter_m / 1e3,
        s.frame.keyhole_at(&circle, aim.target).width / 1e3,
        leads,
        if sign < 0.0 { "retrograde" } else { "prograde" }
    ));

    let impact = s.scenario.impact_epoch();
    let t0 = Instant::now();
    let mut flights = 0usize;
    let mut found: Vec<Crossing> = Vec::new();

    for lead_days in leads {
        let epoch = impact.shifted_by_seconds(-lead_days * 86_400.0);
        let Ok(seed) = ds.nominal().state_at(epoch) else {
            log(&format!(
                "lead {lead_days:8.1} d: the nominal has no state here"
            ));
            continue;
        };
        let Some(dir) = along_track_unit(seed).map(|u| u * sign) else {
            log(&format!("lead {lead_days:8.1} d: no along-track heading"));
            continue;
        };
        // One flight: the impulse, the flyby, and where it lands relative to the
        // circle.
        //
        // **The two ways a flight can fail are not the same thing, and collapsing
        // them truncated this scan.** `Ok(None)` is the 5e8 m scan gate: the pass
        // has swung so wide there is no encounter left to reduce, and the curve
        // really is over — every larger impulse is further out still. `Err` is a
        // propagation failure at one impulse, and the curve continues on the far
        // side of it.
        //
        // At a 50 d lead the retrograde curve passes *through Earth* — the ladder
        // reads b = 762 km at Δv 1.7586, well inside the planet — and the flight
        // fails there. Treating that as the gate stopped the scan at Δv 1.7787 and
        // reported `NOT REACHED`, on a curve the ladder had already flown out to
        // b = 306 654 km at Δv 70. That reads as a reachability finding and is a
        // hole in the middle of the sweep.
        let fly = |dv: f64| -> Result<Option<(nalgebra::Vector2<f64>, f64, f64)>, ()> {
            match ds.evaluate(epoch, dir * dv) {
                Ok(Some(enc)) => {
                    let p = s.frame.project(&enc.b_vector);
                    Ok(Some((p, enc.impact_parameter, circle.signed_distance(p))))
                }
                // Past the scan gate: no encounter, and none at any larger Δv.
                Ok(None) => Ok(None),
                // One bad impulse. Step over it.
                Err(_) => Err(()),
            }
        };

        // --- scan upward for a sign change ON THE REQUESTED BRANCH -----------
        //
        // The line the flown curve traces cuts the circle **twice**, once above
        // the centre (`Plus`) and once below it (`Minus`), and the two are
        // different returns with different doors. Taking the first sign change
        // takes whichever branch the scan's own starting Δv happens to be outside
        // of, which changes with the lead — so the branch is checked and a
        // wrong-branch crossing is logged and scanned past, not accepted.
        let on_branch = |p: nalgebra::Vector2<f64>| match branch {
            CircleBranch::Minus => p.y < circle.center_zeta,
            CircleBranch::Plus => p.y > circle.center_zeta,
        };
        let mut prev: Option<(f64, f64)> = None;
        let mut bracket = None;
        let mut gated_at = None;
        for i in 0..rungs {
            let f = i as f64 / (rungs - 1) as f64;
            let dv = SWEEP_DV_LO * (dv_hi / SWEEP_DV_LO).powf(f);
            flights += 1;
            let (p, d) = match fly(dv) {
                Ok(Some((p, _, d))) => (p, d),
                Ok(None) => {
                    gated_at = Some(dv);
                    break;
                }
                // A hole, not a terminus. `prev` is cleared because a bracket
                // spanning the hole would straddle a discontinuity, and a sign
                // change read across it is not a crossing of the circle.
                Err(()) => {
                    log(&format!(
                        "lead {lead_days:8.1} d:   (no encounter at dv {dv:.6} m/s — stepping over it; this is a failed flight, not the scan gate)"
                    ));
                    prev = None;
                    continue;
                }
            };
            if let Some((dv0, d0)) = prev {
                if d0 * d < 0.0 {
                    // Which side of the centre the crossing is on. Either end of
                    // the bracket answers it — they straddle the circle, not the
                    // centre — and `p` is the one in hand.
                    if on_branch(p) {
                        bracket = Some((dv0, dv));
                        break;
                    }
                    log(&format!(
                        "lead {lead_days:8.1} d:   (crossed the other branch near dv {dv:.6} m/s at zeta {:.1} km — scanning past it)",
                        p.y / 1e3
                    ));
                }
            }
            prev = Some((dv, d));
        }
        let Some((mut lo, mut hi)) = bracket else {
            log(&format!(
                "lead {lead_days:8.1} d: NOT REACHED — {}",
                match gated_at {
                    Some(dv) => format!(
                        "the pass left the 5e8 m scan gate at Δv {dv:.4} m/s before crossing \
                         the circle"
                    ),
                    None => format!(
                        "no sign change up to Δv {dv_hi} m/s (last distance {:.0} km) — raise dvmax= before reading this as unreachable",
                        prev.map(|p| p.1 / 1e3).unwrap_or(f64::NAN)
                    ),
                }
            ));
            continue;
        };

        // --- bisect the crossing ---------------------------------------------
        flights += 1;
        let d_lo = fly(lo).ok().flatten().map(|x| x.2).unwrap_or(f64::NAN);
        let mut best: Option<(f64, nalgebra::Vector2<f64>, f64, f64)> = None;
        for _ in 0..SWEEP_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            flights += 1;
            let Ok(Some((p, b, d))) = fly(mid) else { break };
            if best.as_ref().is_none_or(|x| d.abs() < x.3.abs()) {
                best = Some((mid, p, b, d));
            }
            if d * d_lo > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let Some((dv, point, b_m, residual_m)) = best else {
            log(&format!(
                "lead {lead_days:8.1} d: bisection lost the encounter"
            ));
            continue;
        };
        if !on_branch(point) {
            log(&format!(
                "lead {lead_days:8.1} d: the bisection ended on the WRONG BRANCH                  (zeta {:.1} km vs centre {:.1} km) — not recorded",
                point.y / 1e3,
                circle.center_zeta / 1e3
            ));
            continue;
        }
        let width_m = s.frame.keyhole_at(&circle, point).width;
        log(&format!(
            "lead {lead_days:8.1} d: dv {dv:11.6} m/s -> (xi {:10.1}, zeta {:12.1}) km, \
             b {:10.0} km, door {:8.3} km  [residual {:+.3} km]",
            point.x / 1e3,
            point.y / 1e3,
            b_m / 1e3,
            width_m / 1e3,
            residual_m / 1e3
        ));
        found.push(Crossing {
            lead_days,
            dv,
            point,
            b_m,
            residual_m,
            width_m,
        });
    }

    log(&format!(
        "\n{flights} flights in {:.0} s",
        t0.elapsed().as_secs_f64()
    ));
    if found.len() < 2 {
        log("fewer than two crossings — nothing to compare, and that is the result.");
        return;
    }

    // --- what the spread means -----------------------------------------------
    let xi: Vec<f64> = found.iter().map(|c| c.point.x).collect();
    let lo = xi.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = xi.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let spread = hi - lo;
    // How far apart the extreme crossings are *along the circle*, which is the
    // question's own units: ξ is only a coordinate, the arc is the distance.
    let ang = |c: &Crossing| (c.point.y - circle.center_zeta).atan2(c.point.x);
    let (mut a_lo, mut a_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for c in &found {
        a_lo = a_lo.min(ang(c));
        a_hi = a_hi.max(ang(c));
    }
    let arc = circle.radius * (a_hi - a_lo).abs();
    let worst_residual = found
        .iter()
        .map(|c| c.residual_m.abs())
        .fold(0.0_f64, f64::max);
    let widths: Vec<f64> = found.iter().map(|c| c.width_m).collect();
    let w_lo = widths.iter().cloned().fold(f64::INFINITY, f64::min);
    let w_hi = widths.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    log(&format!(
        "\n=== {resonance} {branch:?}: the crossing across {} leads ===\n\
         xi spans {:.1} .. {:.1} km — a spread of {:.1} km\n\
         arc between the extreme crossings: {:.1} km (on a circle of radius {:.0} km)\n\
         linearised door at the crossing: {:.3} .. {:.3} km ({:.2}x)\n\
         worst search residual: {:.3} km — the spread is {:.0}x that\n\
         b at the crossings: {:.0} .. {:.0} km",
        found.len(),
        lo / 1e3,
        hi / 1e3,
        spread / 1e3,
        arc / 1e3,
        circle.radius / 1e3,
        w_lo / 1e3,
        w_hi / 1e3,
        w_hi / w_lo,
        worst_residual / 1e3,
        spread / worst_residual.max(1.0),
        found.iter().map(|c| c.b_m).fold(f64::INFINITY, f64::min) / 1e3,
        found.iter().map(|c| c.b_m).fold(0.0_f64, f64::max) / 1e3,
    ));
    log(&format!(
        "\nagainst the flown doors: placement errors ran 2.0 .. {LARGEST_PLACEMENT_ERROR_KM:.1} km, \
         and the shipping band is KEYHOLE_PLACEMENT_KM = {SHIPPING_BAND_KM:.0} km.\n  -> {}",
        // The middle branch below is now all but vestigial and that is worth
        // saying rather than leaving for someone to notice: it fires only for a
        // spread between LARGEST_PLACEMENT_ERROR_KM and SHIPPING_BAND_KM, an 18 km
        // window. It used to be the 73 km between 27 and 100. Kept because the two
        // constants are independent and a future door can reopen it, but nothing
        // has been seen to execute it.
        if spread > SHIPPING_BAND_KM * 1e3 {
            "the crossings are further apart than the whole band, so position on the \
             circle is a first-order variable and the doors must be flown at each lead"
        } else if spread > LARGEST_PLACEMENT_ERROR_KM * 1e3 {
            "the crossings are further apart than the largest placement error seen, so \
             the sweep can separate position-on-circle from resonance identity — fly them"
        } else {
            "the crossings are closer together than the placement errors already \
             measured, so this sweep cannot resolve a variation along the circle; the \
             band's flown basis is what it is"
        }
    ));
    for c in &found {
        log(&format!(
            "  lead {:7.1} d  dv {:11.6}  xi {:10.1} km  zeta {:12.1} km",
            c.lead_days,
            c.dv,
            c.point.x / 1e3,
            c.point.y / 1e3
        ));
    }
}

fn report_shot(label: &str, shot: &KeyholeShot, circle: &ResonantCircle) {
    let d = circle.signed_distance(shot.point);
    match shot.flown_return.as_ref() {
        None => log(&format!(
            "{label}: Δv {:.10} → b {:.0} km, {:+.3} km from the circle; no return",
            shot.dv_m_s,
            shot.encounter.impact_parameter / 1e3,
            d / 1e3
        )),
        Some(r) => log(&format!(
            "{label}: Δv {:.10} → b {:.0} km, {:+.3} km from the circle; return {:.1} km \
             (ξ₂ {:.1}, ζ₂ {:.1} km, {})",
            shot.dv_m_s,
            shot.encounter.impact_parameter / 1e3,
            d / 1e3,
            r.distance_m / 1e3,
            r.xi_m.unwrap_or(f64::NAN) / 1e3,
            r.zeta_m.unwrap_or(f64::NAN) / 1e3,
            match r.encounter.as_ref() {
                Some(e) if e.is_hit() => "HIT",
                Some(_) => "miss",
                None => "no reduction",
            }
        )),
    }
}

// ---------------------------------------------------------------------------
// Stage 6 — WHY the placement error depends on the lead: the frame the circle
// is drawn in
// ---------------------------------------------------------------------------
//
// The lead sweep left this open in as many words: "the placement error is now
// known to depend on the deflection lead, and nothing here says why. Three
// points on one circle plus five circles at one lead is a measurement, not a
// model." This stage proposes a model with two terms and tests it.
//
// **Term A — the closed form's own flyby error.** Öpik's construction is an
// instantaneous rotation of `v∞` at Earth's exact heliocentric position. The
// real flyby takes days and happens at the asteroid's position, not Earth's.
// That error is a property of the *circle* and of `b`, and has nothing to do
// with the deflection that delivered the rock there. The five doors flown at
// the campaign's own lead measure it: +19.26, +2.02, −26.75, −2.23 and
// +7.25 km, at Δv from 0.034 to 0.217 m/s. **The signs differ**, which is what
// says it is not a Δv effect.
//
// **Term B — the frame.** A resonant circle is a function of the encounter:
// `c = μ⊕/v∞²`, `θ` (the angle between the incoming asymptote and Earth's
// velocity) and Earth's own heliocentric state. This probe,
// `MissionCore::keyhole_readout` and the drawn map all build that frame from
// the **nominal, undeflected** encounter — and then place the **deflected**
// b-point on it. A deflection changes all three: the rock arrives a little
// faster or slower (`δv∞`), from a slightly different direction (`δθ`), and at
// a different time, so Earth is somewhere else (`δt_CA` — at 30 km/s, an hour
// of arrival slip is 10⁵ km of Earth motion). All three grow with the impulse,
// and the impulse grows roughly as 1/lead. So Term B grows as the lead shortens
// — which is the shape the sweep measured and could not explain.
//
// # What is measured, and the decision rule — written before the run
//
// For one lead, let `d₀` be the flown point's signed distance from the circle
// drawn in the **nominal** frame (what the map reports, and what the door
// campaign published as the placement error) and `d₁` the same point's signed
// distance from the circle drawn in that flight's **own** frame. Their
// difference `δd = d₁ − d₀` is Term B, measured rather than modelled.
//
// If the model is right then **`d₀ + δd` is the same number at every lead** —
// that number being Term A, which belongs to the 3:4 circle and not to any
// plan. Three outcomes:
//
//   * `d₀ + δd` constant to within a door width (~25 km) across 4383, 900 and
//     450 days — the lead dependence is the frame, the mechanism is named, and
//     the fix (draw the circle in the plan's own frame) is available. It is
//     **not applied here**: `keyhole_readout` and `keyhole_circles` are bound to
//     each other by live assertions, and making the frame plan-dependent moves
//     the drawn circles as the player drags the lead slider. That is a design
//     decision, not a side effect of an explanation.
//   * `δd` small at every lead — the frame is *not* the mechanism, the 500 km
//     band's cause is still unknown, and the next experiment is the matched-ξ
//     pair (two leads root-found onto the same ξ, doors flown at both), which is
//     the only thing that separates the lead from the ξ it lands on.
//   * `δd` large but `d₀ + δd` still spreading — the frame is part of it and
//     something else is too. Report the residual; do not fit a law to it.
//
// The closed-form cross-check on `δd`: for a point at angle `φ` on the circle,
// moving the centre by `δζ_c` and the radius by `δR` changes the signed distance
// by `−δR − δζ_c·sin φ`. That is reported beside the exact re-projection, so a
// large `δd` can be read as *which* of the two circle parameters moved rather
// than as one opaque number.
//
// Term B is also split in two, because the three sub-terms are not equally
// suspect: `d_geom` rebuilds the frame from the deflected encounter but keeps
// **Earth's state at the nominal closest approach**, so `d_geom − d₀` is the
// `δv∞`/`δθ` part and `d₁ − d_geom` is the timing part on its own.
//
// ```text
//   probe_keyhole_placement frame [h k branch] [leads...]   # ~1 flight per lead
// ```

/// The 3:4 campaign, by lead: `(lead days, Δv to fly, the door centre the door
/// stage measured there)`.
///
/// The Δv is the refined door floor where one was recorded (4383 d and 900 d)
/// and the `xi_sweep` crossing otherwise. **That distinction matters and is why
/// the door centre is carried separately rather than read off this flight**: at
/// a crossing Δv the flown point sits *on* the circle by construction, so its
/// own `d₀` is ~0 and only `δd` can be read from it, while the recorded door
/// centre supplies the `d₀` the model has to explain. At the two leads where the
/// door's own Δv is known both come from the same flight and the two agree by
/// construction — which is itself the check that reading `δd` at a crossing is
/// legitimate.
///
/// Sources: `door` stage runs of 2026-09-07 (4383, 900, 300 and 200 d) and the `xi_sweep`
/// crossings of the same session (450 d, 150 d). The 150 d entry has no door —
/// the return floors 17 411 km out — so it carries `None` and is flown only to
/// show what the frame is doing where the door has ceased to exist.
///
/// **`same_flight` is the honesty flag on the row.** When the Δv is the door's
/// own floor, `d₀` and `δd` are read from one flight and the row compares like
/// with like. When it is a crossing Δv they are not: `d₀` is a door centre from a
/// flight that no longer exists and `δd` comes from a point elsewhere on the
/// circle. That matters because the frame correction contains `−δζ_c·sin φ`, so
/// it swings with *where on the circle* it was read — the 450 d row's crossing
/// sits at ξ = −15 812 km, φ = −102.2°, against door rows at φ ≈ −85°. The
/// summary reports the spread with and without such rows rather than mixing them
/// silently.
const FLOWN_34_BY_LEAD: &[(f64, f64, Option<f64>, bool)] = &[
    (4383.0, 0.216_548_269_9, Some(19.255e3), true),
    (900.0, 0.932_165_940_6, Some(210.632e3), true),
    (450.0, 1.251_9, Some(467.869e3), false),
    (300.0, 2.879_756_546_8, Some(648.174e3), true),
    (200.0, 2.478_471_262_1, Some(785.988e3), true),
    (150.0, 3.223_5, None, false),
];

fn stage_frame(args: &[String]) {
    let (_, args) = take_lead(args);
    let h: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(3);
    let k: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4);
    let branch = match args.get(3).map(|s| s.to_lowercase()) {
        Some(s) if s.starts_with('p') => CircleBranch::Plus,
        _ => CircleBranch::Minus,
    };
    let resonance = Resonance { h, k };
    let wanted: Vec<f64> = args[4.min(args.len())..]
        .iter()
        .filter_map(|s| s.parse::<f64>().ok())
        .filter(|d| *d > 0.0)
        .collect();

    // One build. The nominal encounter and its frame belong to the undeflected
    // rock, so the circle below is the *same* circle at every lead — which is
    // the whole basis of the comparison.
    let s = setup(None);
    let ds = s.scenario.deflection().expect("deflection");
    let eph = s.scenario.ephemeris().clone();
    let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
    let t_ca0 = ds
        .nominal_encounter_epoch()
        .expect("epoch")
        .expect("an encounter");
    let (r0_km, v0_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca0.as_hifitime())
        .expect("Earth state at the nominal CA");
    let (r0, v0) = (r0_km * 1e3, v0_km * 1e3);

    let Some(c0) = s.frame.resonant_circle(resonance) else {
        eprintln!("{resonance} has no circle in the nominal frame");
        std::process::exit(1);
    };
    let aim = aim_at_resonance(&s.frame, resonance, s.xi, branch).unwrap_or_else(|e| {
        eprintln!("cannot aim at {resonance}: {e}");
        std::process::exit(1);
    });
    let sign = if aim.target.y < 0.0 { -1.0 } else { 1.0 };
    log(&format!(
        "\n{resonance} {branch:?} in the NOMINAL frame: centre ζ {:.3} km, radius {:.3} km\n\
         nominal encounter: v∞ {:.3} m/s, θ {:.6}°, CA at {}\n\
         {} nudge; one flight per lead, no doors flown and no refinement",
        c0.center_zeta / 1e3,
        c0.radius / 1e3,
        s.frame.v_inf,
        s.frame.theta().to_degrees(),
        t_ca0.as_hifitime(),
        if sign < 0.0 { "retrograde" } else { "prograde" }
    ));

    let impact = s.scenario.impact_epoch();
    let mut rows: Vec<(f64, f64, f64, f64, f64, f64, bool)> = Vec::new();
    let t0 = Instant::now();

    for &(lead_days, dv, door_centre, same_flight) in FLOWN_34_BY_LEAD {
        if !wanted.is_empty() && !wanted.iter().any(|w| (w - lead_days).abs() < 0.5) {
            continue;
        }
        // The campaign's own lead is `epoch0()` **exactly**, not `impact − 4383 d`.
        // The two differ by hours, and at this leverage (b is very nearly
        // proportional to the lead) hours are tens of kilometres — the same size
        // as the 19.3 km this row exists to explain.
        let epoch = if (lead_days - s.default_lead_days).abs() < 1.0 {
            s.scenario.epoch0()
        } else {
            impact.shifted_by_seconds(-lead_days * 86_400.0)
        };
        let Ok(seed) = ds.nominal().state_at(epoch) else {
            log(&format!(
                "lead {lead_days:8.1} d: the nominal has no state here"
            ));
            continue;
        };
        let Some(dir) = along_track_unit(seed).map(|u| u * sign) else {
            log(&format!("lead {lead_days:8.1} d: no along-track heading"));
            continue;
        };
        let aiming = KeyholeAiming {
            frame: &s.frame,
            earth_radius_m: s.nominal.earth_radius,
            deflection_epoch: epoch,
            direction: dir,
        };
        let opts = KeyholeShotOptions::default().widened_for_aim(aim.impact_parameter_m);
        let shot = match fly_keyhole_shot(&s.scenario, &ds, aiming, dv, resonance, &opts) {
            Ok(shot) => shot,
            Err(e) => {
                log(&format!("lead {lead_days:8.1} d: {e}"));
                continue;
            }
        };

        // Earth's state at *this flight's own* closest approach.
        let (r1_km, v1_km) = eph
            .state_km_s(EARTH_J2000, SUN_J2000, shot.first_epoch.as_hifitime())
            .expect("Earth state at the deflected CA");
        let (r1, v1) = (r1_km * 1e3, v1_km * 1e3);
        let dt_ca = shot.first_epoch.tdb_seconds_past_j2000() - t_ca0.tdb_seconds_past_j2000();

        // Three frames: the map's, this flight's geometry on the map's clock, and
        // this flight's own.
        let f_geom = OpikFrame::new(&shot.encounter, r0, v0, mu_sun).expect("geometry frame");
        let f_full = OpikFrame::new(&shot.encounter, r1, v1, mu_sun).expect("own frame");
        let (Some(c_geom), Some(c_full)) = (
            f_geom.resonant_circle(resonance),
            f_full.resonant_circle(resonance),
        ) else {
            log(&format!(
                "lead {lead_days:8.1} d: {resonance} has no circle in the rebuilt frame"
            ));
            continue;
        };

        let p0 = s.frame.project(&shot.encounter.b_vector);
        let d0 = c0.signed_distance(p0);
        let d_geom = c_geom.signed_distance(f_geom.project(&shot.encounter.b_vector));
        let d_full = c_full.signed_distance(f_full.project(&shot.encounter.b_vector));

        let phi = (p0.y - c0.center_zeta).atan2(p0.x);
        let d_zeta_c = c_full.center_zeta - c0.center_zeta;
        let d_radius = c_full.radius - c0.radius;
        let predicted = -d_radius - d_zeta_c * phi.sin();

        // How far out of the nominal b-plane the deflected b-vector has tipped —
        // the part the map's projection silently drops.
        let out_of_plane = shot.encounter.b_vector.dot(&s.frame.eta_hat);

        log(&format!(
            "\n--- lead {lead_days:.0} d, Δv {dv:.10} m/s {} ---\n\
             flight: b {:.1} km, point (ξ {:.1}, ζ {:.1}) km, φ {:.3}°, return {}\n\
             encounter shift: δv∞ {:+.4} m/s ({:+.3e} rel), δθ {:+.3e}°, δt_CA {:+.1} s \
             ({:+.2} h → Earth moved {:.0} km), b out of the nominal plane {:+.1} km\n\
             circle shift (own frame vs map's): δζ_c {:+.3} km, δR {:+.3} km\n\
             signed distance: map's frame d₀ {:+.3} km | geometry only {:+.3} km \
             ({:+.3} km) | own frame d₁ {:+.3} km (δd {:+.3} km)\n\
             closed form −δR − δζ_c·sin φ = {:+.3} km against that exact δd",
            if sign < 0.0 { "retrograde" } else { "prograde" },
            shot.encounter.impact_parameter / 1e3,
            p0.x / 1e3,
            p0.y / 1e3,
            phi.to_degrees(),
            match shot.flown_return.as_ref() {
                None => "none in the gate".to_string(),
                Some(r) => format!(
                    "{:.1} km ({})",
                    r.distance_m / 1e3,
                    match r.encounter.as_ref() {
                        Some(e) if e.is_hit() => "HIT",
                        Some(_) => "miss",
                        None => "no reduction",
                    }
                ),
            },
            f_full.v_inf - s.frame.v_inf,
            (f_full.v_inf - s.frame.v_inf) / s.frame.v_inf,
            (f_full.theta() - s.frame.theta()).to_degrees(),
            dt_ca,
            dt_ca / 3600.0,
            dt_ca.abs() * v0.norm() / 1e3,
            out_of_plane / 1e3,
            d_zeta_c / 1e3,
            d_radius / 1e3,
            d0 / 1e3,
            d_geom / 1e3,
            (d_geom - d0) / 1e3,
            d_full / 1e3,
            (d_full - d0) / 1e3,
            predicted / 1e3,
        ));

        // Term B is read at whatever point was flown; Term A needs the door
        // centre, which at a crossing Δv is not this flight's own `d₀`.
        let delta_d = d_full - d0;
        match door_centre {
            Some(centre) => rows.push((
                lead_days,
                centre,
                delta_d,
                centre + delta_d,
                d_geom - d0,
                d_full - d_geom,
                same_flight,
            )),
            None => log(
                "  no door was ever found at this lead, so there is no d₀ to correct — the \
                 frame shift above is reported for scale only.",
            ),
        }
    }

    if rows.is_empty() {
        log("\nno lead produced a usable row.");
        return;
    }
    log(&format!(
        "\n=== {resonance} {branch:?}: IS THE LEAD DEPENDENCE THE FRAME? === \
         ({} rows in {:.0} s)\n\n\
         | lead | door centre d₀ | frame term δd | d₀ + δd (Term A) | of which v∞/θ | \
         of which timing | d₀ and δd from |\n|---|---|---|---|---|---|---|",
        rows.len(),
        t0.elapsed().as_secs_f64()
    ));
    for (lead, d0, dd, term_a, geom, timing, same) in &rows {
        log(&format!(
            "| {lead:.0} d | {:+.3} km | {:+.3} km | **{:+.3} km** | {:+.3} km | {:+.3} km | {} |",
            d0 / 1e3,
            dd / 1e3,
            term_a / 1e3,
            geom / 1e3,
            timing / 1e3,
            if *same {
                "one flight"
            } else {
                "**two flights**"
            }
        ));
    }
    let spread = |v: Vec<f64>| {
        v.iter().cloned().fold(f64::MIN, f64::max) - v.iter().cloned().fold(f64::MAX, f64::min)
    };
    let clean: Vec<&(f64, f64, f64, f64, f64, f64, bool)> = rows.iter().filter(|r| r.6).collect();
    let spread_before = spread(rows.iter().map(|r| r.1).collect());
    let spread_after = spread(rows.iter().map(|r| r.3).collect());
    let clean_before = spread(clean.iter().map(|r| r.1).collect());
    let clean_after = spread(clean.iter().map(|r| r.3).collect());
    log(&format!(
        "\n                                                    all {} rows | the {} \
         same-flight rows\n\
         spread of the published placement error :   {:9.1} km | {:9.1} km\n\
         spread once each flight's own frame is used:{:9.1} km | {:9.1} km\n\
         the door this is measured against is ~{:.1} km wide.",
        rows.len(),
        clean.len(),
        spread_before / 1e3,
        clean_before / 1e3,
        spread_after / 1e3,
        clean_after / 1e3,
        s.frame.keyhole_at(&c0, aim.target).width / 1e3
    ));
    log(
        "\nRead the same-flight column, and read the middle column of the table rather than \
         any ratio: the claim under test is that `d₀ + δd` is ONE number belonging to this \
         circle. A spread that stays above a door width means the frame is part of the \
         mechanism and something else is too — and the same-flight column is the one that \
         says so without a mixed-provenance row to argue about.",
    );
}

// ---------------------------------------------------------------------------
// Stage 7 — can the "several doors in the band" register fire on a real flight?
// ---------------------------------------------------------------------------
//
// `KEYHOLE_PLACEMENT_KM` grew to 500 km and the panel gained a third register:
// when more than one door falls inside the band it declines to name a
// resonance. That register ships with a unit test behind it and **has never
// been observed on real physics**. The one plan flown to a return reports
// `doors_in_band = 1`, and the 83.6 km figure that motivated the register is the
// tightest pair *anywhere* in that ξ's census — at some other impact parameter
// entirely, not where that plan sits. The constant's own doc now says the
// crowded case is "inferred from closed-form geometry and has not yet fired on
// a flown plan". This stage goes looking for one.
//
// # Why a coarse Δv sweep cannot find it, and what this does instead
//
// The crowded windows are the width of the band either side of a pair of
// circles — of order a thousand kilometres of `b` — while a Δv ladder samples
// `b` in steps of tens of thousands. Sweeping and hoping to land in one is a
// lottery. So the search is in three parts:
//
//   1. **fly a coarse curve** — the plan's `(ξ, ζ)` at ~40 impulses per
//      direction, real flights, no returns and no interpolation of physics;
//   2. **scan it densely in closed form** — interpolate that curve and count
//      `doors_within_band` at tens of thousands of points along it, which costs
//      nothing and finds every window the curve passes through;
//   3. **fly the windows** — a real flight at each candidate Δv, and a bisection
//      toward the window's own `b` if the first shot lands outside it. Only a
//      flown point counts. The interpolation picks *where to look*; it is never
//      the evidence.
//
// The census is the **readout's**, not this probe's survey census: 2..=7 years,
// `k ≤ 24`, `b ≤ 60 × capture radius`. A window among circles the frontend does
// not draw would prove nothing about the frontend's register.
//
// **And the curve is gated on the deflected pass being a miss.** The first run of
// this stage reported the register firing at three leads — and every one of the
// plans it found sat at `b` between 4 593 and 9 526 km against an 11 311 km
// capture radius, i.e. **still an impact**. Circles crowd near Earth because that
// is where every resonance's circle has to pass, so an ungated search finds its
// answer there every time and the answer is worthless: a plan that has not yet
// turned the hit into a miss has no keyhole question to get wrong. Only points
// with `!enc.is_hit()` are counted, and the rejected ones are reported rather
// than silently dropped — the count of them is the reason this gate exists.
//
// # The decision rule, written before the run
//
//   * a real flight at a dialable `(lead, Δv)` reports **two or more doors in
//     the band** — the register fires on real physics, and that plan becomes a
//     binding test;
//   * the closed-form scan finds windows but no flight lands inside one — the
//     window is real and narrower than the search can steer to; report it as
//     unconfirmed with the residual `b` and do not upgrade the doc's claim;
//   * no window anywhere on the dialable curve at any lead swept — the register
//     is unreachable on this rock, "inferred from geometry" stands, and now
//     there is a measured search behind that sentence instead of a guess.
//
// # What this can and cannot claim
//
// A flight that reports two doors in the band says **the panel cannot name one
// resonance there**. It does *not* say two impact keyholes exist at that point:
// that needs two returns flown to Earth, two edge bisections, ~25 minutes per
// door, and it is a different question. The register is about what the map is
// entitled to assert, and that is what is measured here.
//
// ```text
//   probe_keyhole_placement crowding [lead=<days>] [max_years] [band=<km>]
// ```

/// Δv span the coarse curve covers, m/s, and its resolution.
///
/// The low end is `sim.gd`'s `DV_MIN`; the high end is past where the deflected
/// pass leaves the shipping 5e8 m scan gate at every lead swept, which the sweep
/// reports rather than assumes (`DV_MAX` is 300, so the gate binds long before
/// the slider does). 40 rungs over that span is ~9 % in Δv per rung, which is
/// what makes the log interpolation between them good to the ~1 % of `b` needed
/// to steer into a thousand-kilometre window.
const CROWD_DV_LO: f64 = 0.1;
const CROWD_DV_HI: f64 = 4.0;
const CROWD_RUNGS: usize = 40;
/// Real flights spent per candidate window in stage 3. Four is enough to cross a
/// window whose ends the interpolation placed to ~1 % of `b`, and the whole point
/// is that these are flights: a window this many misses is reported as
/// unconfirmed rather than argued into an answer.
const CROWD_CONFIRM_SAMPLES: usize = 4;
/// Points of the interpolated curve the closed-form scan visits per direction.
/// Free — no flights — so this is set by the narrowest window worth finding
/// rather than by cost.
const CROWD_SCAN_POINTS: usize = 40_000;

/// One flown rung of the plan curve.
#[derive(Clone, Copy)]
struct CurvePoint {
    dv: f64,
    point: nalgebra::Vector2<f64>,
    b_m: f64,
    /// The deflected pass misses Earth. A crowded point that still impacts is not
    /// an answer to this question — see the stage doc.
    is_miss: bool,
}

fn stage_crowding(args: &[String]) {
    let (lead, args) = take_lead(args);
    let max_years: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(7);
    let band_km: f64 = args
        .iter()
        .filter_map(|a| a.strip_prefix("band=").and_then(|s| s.parse::<f64>().ok()))
        .next()
        .unwrap_or(500.0);
    let band = band_km * 1e3;

    let s = setup(Some(lead.unwrap_or(900.0)));
    let ds = s.scenario.deflection().expect("deflection");
    // The frontend's census, exactly: `MissionCore::keyhole_readout` calls
    // `resonant_circles(2..=max_years, 24, 60 × capture_radius)`.
    let circles = s.frame.resonant_circles(
        2..=max_years.max(2),
        MAX_REVOLUTIONS,
        B_MAX_CAPTURE_RADII * s.nominal.capture_radius,
    );
    log(&format!(
        "\n{} circles in the readout's own census (2..={} yr, k ≤ {}, b ≤ {:.0} km); \
         band {:.0} km\nlead {:.1} d, Δv {:.2} .. {:.2} m/s in {} rungs per direction",
        circles.len(),
        max_years.max(2),
        MAX_REVOLUTIONS,
        B_MAX_CAPTURE_RADII * s.nominal.capture_radius / 1e3,
        band_km,
        s.lead_days,
        CROWD_DV_LO,
        CROWD_DV_HI,
        CROWD_RUNGS
    ));

    let seed = ds
        .nominal()
        .state_at(s.deflection_epoch)
        .expect("nominal state at the impulse epoch");
    let prograde = along_track_unit(seed).expect("along-track");
    let t0 = Instant::now();
    let mut flights = 0usize;

    // One flight: impulse, flyby, b-plane point. `None` is the scan gate.
    let fly = |dv_signed: f64| -> Option<CurvePoint> {
        let dir = prograde * dv_signed.signum();
        match ds.evaluate(s.deflection_epoch, dir * dv_signed.abs()) {
            Ok(Some(enc)) => Some(CurvePoint {
                dv: dv_signed,
                point: s.frame.project(&enc.b_vector),
                b_m: enc.impact_parameter,
                is_miss: !enc.is_hit(),
            }),
            _ => None,
        }
    };
    let count_at = |p: nalgebra::Vector2<f64>| s.frame.doors_within_band(&circles, p, band);
    // The **second** smallest margin in the census — the number that decides
    // whether a band can name one resonance, and the one a negative result has to
    // quote. `f64::INFINITY` when the census has fewer than two circles.
    let runner_up = |p: nalgebra::Vector2<f64>| -> f64 {
        let mut m: Vec<f64> = s
            .frame
            .keyhole_proximities(&circles, p)
            .into_iter()
            .map(|k| k.margin())
            .collect();
        m.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        m.get(1).copied().unwrap_or(f64::INFINITY)
    };
    let rows_at = |p: nalgebra::Vector2<f64>| -> Vec<(String, f64, f64, f64)> {
        let mut v: Vec<(String, f64, f64, f64)> = s
            .frame
            .keyhole_proximities(&circles, p)
            .into_iter()
            .map(|k| {
                (
                    format!("{}:{}", k.circle.resonance.h, k.circle.resonance.k),
                    k.signed_distance,
                    k.keyhole.width,
                    k.margin(),
                )
            })
            .collect();
        v.sort_by(|a, b| a.3.partial_cmp(&b.3).expect("finite"));
        v
    };

    let mut best_flown: Option<(CurvePoint, usize)> = None;
    let mut windows: Vec<(f64, f64, f64, usize)> = Vec::new();
    // The nearest this lead's curve ever comes to crowded, over misses only:
    // `(second-smallest margin, Δv, point)`. A "not reachable" that does not say
    // by how much is not a measurement.
    let mut closest = (f64::INFINITY, f64::NAN, nalgebra::Vector2::zeros());

    for sign in [-1.0_f64, 1.0] {
        // --- 1. the coarse curve, flown --------------------------------------
        let mut curve: Vec<CurvePoint> = Vec::new();
        for i in 0..CROWD_RUNGS {
            let t = i as f64 / (CROWD_RUNGS - 1) as f64;
            let dv = CROWD_DV_LO * (CROWD_DV_HI / CROWD_DV_LO).powf(t);
            flights += 1;
            match fly(sign * dv) {
                Some(p) => curve.push(p),
                None => {
                    log(&format!(
                        "  {} curve: left the 5e8 m scan gate at Δv {dv:.3} m/s after {} rungs",
                        if sign < 0.0 { "retrograde" } else { "prograde" },
                        curve.len()
                    ));
                    break;
                }
            }
        }
        if curve.len() < 2 {
            continue;
        }
        let impacting = curve.iter().filter(|p| !p.is_miss).count();
        let flown_counts: Vec<usize> = curve
            .iter()
            .map(|p| if p.is_miss { count_at(p.point) } else { 0 })
            .collect();
        for p in curve.iter().filter(|p| p.is_miss) {
            let r = runner_up(p.point);
            if r < closest.0 {
                closest = (r, p.dv, p.point);
            }
        }
        log(&format!(
            "  {} curve: {} rungs flown ({impacting} still impacting, not counted), \
             b {:.0} .. {:.0} km, ξ {:.0} .. {:.0} km; doors in band at the rungs \
             themselves: max {}",
            if sign < 0.0 { "retrograde" } else { "prograde" },
            curve.len(),
            curve.first().expect("nonempty").b_m / 1e3,
            curve.last().expect("nonempty").b_m / 1e3,
            curve.first().expect("nonempty").point.x / 1e3,
            curve.last().expect("nonempty").point.x / 1e3,
            flown_counts.iter().max().copied().unwrap_or(0)
        ));
        for (p, c) in curve.iter().zip(&flown_counts) {
            if *c >= 2 && best_flown.as_ref().is_none_or(|(_, bc)| c > bc) {
                best_flown = Some((*p, *c));
            }
        }

        // --- 2. the free scan between the rungs ------------------------------
        //
        // Linear in log Δv between the two bracketing flown rungs. The curve is
        // smooth and the rungs are ~9 % apart, so this is an interpolation of
        // *flown* points and never an extrapolation — but it is still only a
        // pointer to where to fly next, never evidence.
        // `None` where either bracketing rung still impacts: the miss/impact
        // boundary is a real edge of the interpolation's validity, not a place to
        // average across.
        let at = |dv_abs: f64| -> Option<nalgebra::Vector2<f64>> {
            let i = curve
                .partition_point(|r| r.dv.abs() < dv_abs)
                .clamp(1, curve.len() - 1);
            let (lo, hi) = (curve[i - 1], curve[i]);
            if !(lo.is_miss && hi.is_miss) {
                return None;
            }
            let t = (dv_abs.ln() - lo.dv.abs().ln()) / (hi.dv.abs().ln() - lo.dv.abs().ln());
            Some(lo.point + t * (hi.point - lo.point))
        };
        let lo_dv = curve.first().expect("nonempty").dv.abs();
        let hi_dv = curve.last().expect("nonempty").dv.abs();
        let mut run: Option<(f64, f64, usize)> = None;
        for i in 0..CROWD_SCAN_POINTS {
            let t = i as f64 / (CROWD_SCAN_POINTS - 1) as f64;
            let dv = lo_dv * (hi_dv / lo_dv).powf(t);
            let Some(p) = at(dv) else {
                if let Some(r) = run.take() {
                    windows.push((sign * r.0, sign * r.1, 0.0, r.2));
                }
                continue;
            };
            let c = count_at(p);
            let r = runner_up(p);
            if r < closest.0 {
                closest = (r, sign * dv, p);
            }
            match (&mut run, c >= 2) {
                (None, true) => run = Some((dv, dv, c)),
                (Some(r), true) => {
                    r.1 = dv;
                    r.2 = r.2.max(c);
                }
                (Some(r), false) => {
                    windows.push((sign * r.0, sign * r.1, 0.0, r.2));
                    run = None;
                }
                (None, false) => {}
            }
        }
        if let Some(r) = run {
            windows.push((sign * r.0, sign * r.1, 0.0, r.2));
        }
    }

    log(&format!(
        "\n{flights} flights in {:.0} s. Closed-form scan found {} window(s) where the \
         interpolated curve has ≥ 2 doors in the band.",
        t0.elapsed().as_secs_f64(),
        windows.len()
    ));
    // Widest first: a wide window is the one a real flight can actually be
    // steered into, and the narrow ones are where the interpolation is least
    // trustworthy anyway.
    windows.sort_by(|a, b| {
        ((b.1 - b.0).abs())
            .partial_cmp(&(a.1 - a.0).abs())
            .expect("finite")
    });
    for (lo, hi, _, c) in windows.iter().take(8) {
        log(&format!(
            "  Δv {:+.6} .. {:+.6} m/s  (width {:.2e} m/s), up to {c} doors",
            lo,
            hi,
            (hi - lo).abs()
        ));
    }

    // --- 3. fly the windows --------------------------------------------------
    //
    // The interpolation says *where* to look and is never the evidence, so each
    // candidate window is sampled at evenly spaced interior points and every one
    // of them is a real flight. Spread across the window rather than bisected
    // toward its middle: the real curve is offset from the interpolated one by an
    // unknown amount, so there is no measured quantity to bisect on, and pretending
    // otherwise would be a search that reports a bracket as an answer.
    let mut confirmed: Option<(f64, CurvePoint, usize)> = None;
    'windows: for (lo, hi, _, _) in windows.iter().take(4) {
        for i in 1..=CROWD_CONFIRM_SAMPLES {
            let t = i as f64 / (CROWD_CONFIRM_SAMPLES + 1) as f64;
            let dv = lo + t * (hi - lo);
            flights += 1;
            let Some(p) = fly(dv) else {
                log(&format!("  candidate Δv {dv:+.6}: left the scan gate"));
                continue;
            };
            let c = if p.is_miss { count_at(p.point) } else { 0 };
            log(&format!(
                "  candidate Δv {dv:+.10} m/s → b {:.1} km, (ξ {:.1}, ζ {:.1}) km, {}, \
                 doors in the {band_km:.0} km band: {c}",
                p.b_m / 1e3,
                p.point.x / 1e3,
                p.point.y / 1e3,
                if p.is_miss {
                    "a miss"
                } else {
                    "STILL AN IMPACT"
                }
            ));
            if c >= 2 {
                confirmed = Some((dv, p, c));
                break 'windows;
            }
        }
    }

    log(&format!(
        "\n=== CROWDED REGISTER, LEAD {:.0} d === ({flights} flights in {:.0} s)",
        s.lead_days,
        t0.elapsed().as_secs_f64()
    ));
    match confirmed.or(best_flown.map(|(p, c)| (p.dv, p, c))) {
        Some((dv, p, c)) => {
            log(&format!(
                "FIRES on a flown plan: lead {:.0} d, Δv {:+.10} m/s → b {:.1} km at \
                 (ξ {:.1}, ζ {:.1}) km, {c} doors inside the {band_km:.0} km band.\n\
                 The panel cannot name one resonance here. This does NOT say {c} impact \
                 keyholes exist at that point — that needs a return flown and both edges \
                 bisected per door, which this stage does not do.\n\
                 The rows, by margin:",
                s.lead_days,
                dv,
                p.b_m / 1e3,
                p.point.x / 1e3,
                p.point.y / 1e3
            ));
            log(&format!(
                "    (b {:.1} km against a {:.1} km capture radius, so this really is a \
                 miss; the runner-up margin's best anywhere on this curve is {:.1} km)",
                p.b_m / 1e3,
                s.nominal.capture_radius / 1e3,
                closest.0 / 1e3
            ));
            for (name, d, w, m) in rows_at(p.point).iter().take(6) {
                log(&format!(
                    "    {name:>6}  d {:+10.1} km  width {:8.3} km  margin {:+10.1} km{}",
                    d / 1e3,
                    w / 1e3,
                    m / 1e3,
                    if *m <= band { "  ← in the band" } else { "" }
                ));
            }
        }
        None if windows.is_empty() => log(&format!(
            "NOT REACHABLE at this lead: no point of the dialable curve that is a MISS, \
             flown or interpolated, has two doors inside the band.\n\
             How close it came: the runner-up door's smallest margin anywhere on the \
             curve is {:.1} km, at Δv {:+.6} m/s, (ξ {:.1}, ζ {:.1}) km — so a band of \
             {:.0} km would have been needed here against the {band_km:.0} km asked for.",
            closest.0 / 1e3,
            closest.1,
            closest.2.x / 1e3,
            closest.2.y / 1e3,
            closest.0 / 1e3
        )),
        None => log(
            "UNCONFIRMED: the closed-form scan finds windows on this curve but no flight \
             landed inside one. The window is real and narrower than this search can steer \
             to; the doc's claim must not be upgraded on it.",
        ),
    }
}

// ---------------------------------------------------------------------------
// Stage 8 — the outgoing orbit itself: is the map's `a'` wrong, or is `a' = a_res`
// the wrong condition?
// ---------------------------------------------------------------------------

/// Fly a recorded door centre and measure the semi-major axis the flyby **really**
/// leaves the rock on, against the two things the map assumes about it.
///
/// Every previous stage measured the door's *position* and compared candidate
/// explanations for where it sits. All of them worked inside the closed form: the
/// `frame` stage rebuilt `c`, `θ` and Earth's clock and re-drew the circle, and
/// made the spread worse. None of them ever asked the propagator what orbit the
/// flyby actually produced. That number has never been measured in this repo —
/// [`KeyholeShot::a_prime_m`] is the *closed form's* answer at the flown point,
/// not the flown one.
///
/// Two claims are folded into one circle, and this stage separates them:
///
/// 1. **the prediction** — at a b-plane point `p`, the flyby leaves the rock on
///    the `a'` that [`OpikFrame::post_encounter_semi_major_axis`] says it does;
/// 2. **the condition** — landing on `a' = a_res` is what produces a resonant
///    return impact `h` years later.
///
/// The circle is drawn where claim 1 says claim 2 is met. A door 786 km away from
/// it means at least one of them is false there.
///
/// **The decision rule, written before the run.** Let `E` be the recorded
/// placement error of the row (19.3 km at 4383 d, 786.0 km at 200 d) and let
/// `∇a'·n̂` convert an `a'` difference into a distance along the circle's outward
/// normal — exact, not a proxy, because the circles *are* the level sets of `a'`,
/// so `∇a'` is normal to them.
///
/// - If `(a_true − a_res)/∇a'·n̂` is small against `E` at both leads, while
///   `(a_true − a'_closed)/∇a'·n̂ ≈ −d₀` and tracks the 34× ratio between them:
///   **the condition is right and the prediction is biased.** The map draws the
///   circle in the wrong place because it mispredicts the outgoing orbit, and the
///   next question is which of the closed form's approximations does it.
/// - If `(a_true − a'_closed)/∇a'·n̂` is small against `E` while
///   `(a_true − a_res)/∇a'·n̂ ≈ d₀`: **the prediction is right and the condition is
///   wrong.** A resonant return does not need `a = a_res`, and the missing term is
///   phase/timing — the next question is what sets the offset.
/// - A third outcome (both small, with `d₀` large) is not a physical answer, it is
///   a broken conversion. The stage prints `(a'_closed − a_res)/∇a'·n̂` beside `d₀`
///   as the arithmetic identity check; if those two disagree, nothing else on the
///   row may be read.
///
/// **The sampling convention is the risk, so it is measured rather than chosen.**
/// An osculating heliocentric `a` after the flyby is not convention-free: the
/// planets wander it, and the signal being discriminated is only
/// `∇a'·n̂ × 800 km`. Three epochs — 10, 30 and 90 days past closest approach, i.e.
/// ~3, ~9 and ~27 Earth Hill radii out — and the **spread between them is printed
/// in the same kilometres as the answer**. If the spread reaches a third of `E`,
/// this stage cannot discriminate at that lead and says so instead of picking the
/// epoch that gives a tidy answer.
///
/// The `r ≈ R⊕ₒᵣᵦ` column is free once a flight exists.
/// [`OpikFrame::cos_theta_out_for_semi_major_axis`] evaluates vis-viva at
/// **Earth's** heliocentric distance while the rock is 140 000–150 000 km away,
/// and `gradient_semi_major_axis`'s own doc calls itself "the one output the
/// `r ≈ R⊕ₒᵣᵦ` approximation does *not* corrupt to first order" — an admission
/// that `a'` is corrupted by it. So the stage re-draws the circle with `r` set to
/// the rock's own heliocentric distance at closest approach and reports where that
/// circle then sits relative to the flown door. It costs no extra flight, and it
/// may well come back negative: the offset's radial part varies with where on the
/// circle the plan lands, and ξ is one of the variables already shown *not* to
/// order the five errors.
///
/// Runs the two extremes only by default — 4383 d (error 19.3 km) and 200 d
/// (786.0 km). If the gap does not track the 34× between them, the middle rows
/// cannot rescue it. The 450 d and 150 d rows are excluded outright: the first is
/// the mixed-provenance row (`same_flight = false`), the second has no door.
/// One row of the ingredient swap: `(lead days, ∇a'·n̂, the bias to explain, and then
/// the km-equivalent each of `Ŝ`, `v∞`, Earth's position and Earth's velocity moves
/// the prediction on its own, all four at once, how far the flown b-vector lies out
/// of the nominal plane, and how much of the `Ŝ` term is only the `|b|` that drops).
type IngredientRow = (f64, f64, f64, f64, f64, f64, f64, f64, f64, f64);

/// One row of the two-leg split: `(lead days, ξ of the flown point, the incoming error
/// in the nominal frame and in the flight's own, the outgoing error in each, the true
/// change across the encounter and the change predicted by each frame, and the
/// incoming mean's own error bar)`. Kilometres throughout.
type LegRow = (f64, f64, f64, f64, f64, f64, f64, f64, f64, f64);

/// One row of the repair: `(lead days, the door's distance from the circle as the map
/// draws it today, the same distance if the circle were placed on the revolution-mean
/// change in `a`, the same on a single osculating sample, the baseline's error bar)`.
type RepairRow = (f64, f64, f64, f64, f64);

fn stage_outgoing(args: &[String]) {
    let (_, args) = take_lead(args);
    let h: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(3);
    let k: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4);
    let branch = match args.get(3).map(|s| s.to_lowercase()) {
        Some(s) if s.starts_with('p') => CircleBranch::Plus,
        _ => CircleBranch::Minus,
    };
    let resonance = Resonance { h, k };
    let wanted: Vec<f64> = args[4.min(args.len())..]
        .iter()
        .filter_map(|s| s.parse::<f64>().ok())
        .collect();
    let leads: Vec<f64> = if wanted.is_empty() {
        vec![4383.0, 200.0]
    } else {
        wanted
    };

    let s = setup(None);
    let ds = s.scenario.deflection().expect("deflection");
    let eph = s.scenario.ephemeris().clone();
    let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
    let a_res = resonance.semi_major_axis_m();

    let Some(c0) = s.frame.resonant_circle(resonance) else {
        eprintln!("{resonance} has no circle in the nominal frame");
        std::process::exit(1);
    };
    let aim = aim_at_resonance(&s.frame, resonance, s.xi, branch).unwrap_or_else(|e| {
        eprintln!("cannot aim at {resonance}: {e}");
        std::process::exit(1);
    });
    let sign = if aim.target.y < 0.0 { -1.0 } else { 1.0 };
    log(&format!(
        "\n{resonance} {branch:?}: a_res {:.9} AU, circle centre ζ {:.3} km, radius {:.3} km",
        a_res / AU_M,
        c0.center_zeta / 1e3,
        c0.radius / 1e3
    ));
    log(&format!(
        "flying recorded door centres; {} nudge; osculating a sampled at CA+10/30/90 d",
        if sign < 0.0 { "retrograde" } else { "prograde" }
    ));

    let impact = s.scenario.impact_epoch();
    let scan = ScanOptions {
        max_sample_dt: 6.0 * 3600.0,
        time_tol_seconds: 1.0e-3,
        max_distance: Some(5.0e8),
    };
    let t0 = Instant::now();
    let mut split: Vec<IngredientRow> = Vec::new();
    let mut inbound: Vec<LegRow> = Vec::new();
    let mut repair: Vec<RepairRow> = Vec::new();

    for &(lead_days, dv, door_centre, same_flight) in FLOWN_34_BY_LEAD {
        if !leads.iter().any(|w| (w - lead_days).abs() < 0.5) {
            continue;
        }
        let Some(recorded) = door_centre else {
            log(&format!(
                "lead {lead_days:8.1} d: no door recorded — skipped"
            ));
            continue;
        };
        if !same_flight {
            log(&format!(
                "lead {lead_days:8.1} d: mixed-provenance row — skipped"
            ));
            continue;
        }
        let epoch = if (lead_days - s.default_lead_days).abs() < 1.0 {
            s.scenario.epoch0()
        } else {
            impact.shifted_by_seconds(-lead_days * 86_400.0)
        };
        let Ok(seed) = ds.nominal().state_at(epoch) else {
            log(&format!(
                "lead {lead_days:8.1} d: the nominal has no state here"
            ));
            continue;
        };
        let Some(dir) = along_track_unit(seed).map(|u| u * sign) else {
            log(&format!("lead {lead_days:8.1} d: no along-track heading"));
            continue;
        };

        let (clock, _) = match ds.deflected_trajectory(epoch, dir * dv) {
            Ok(t) => t,
            Err(e) => {
                log(&format!("lead {lead_days:8.1} d: {e}"));
                continue;
            }
        };
        let earth = EphemerisPerturber::new(eph.clone(), EARTH_J2000);
        let ca = match closest_approach(&clock, &earth, scan) {
            Ok(Some(ca)) => ca,
            Ok(None) => {
                log(&format!("lead {lead_days:8.1} d: no encounter in the gate"));
                continue;
            }
            Err(e) => {
                log(&format!("lead {lead_days:8.1} d: {e}"));
                continue;
            }
        };
        let enc = match ca.b_plane(s.frame.mu_earth, s.nominal.earth_radius) {
            Ok(e) => e,
            Err(e) => {
                log(&format!("lead {lead_days:8.1} d: {e}"));
                continue;
            }
        };
        let p = s.frame.project(&enc.b_vector);
        let d0 = c0.signed_distance(p);

        // The conversion, exact rather than a proxy: the circles are the level
        // sets of a', so ∇a' lies along the circle's outward normal.
        let n_hat = (p - Vector2::new(0.0, c0.center_zeta)).normalize();
        let grad = s.frame.gradient_semi_major_axis(p);
        let grad_n = grad.dot(&n_hat);
        let a_closed = s.frame.post_encounter_semi_major_axis(p);
        let identity_km = (a_closed - a_res) / grad_n / 1e3;

        // The rock's own heliocentric state at closest approach, and at three
        // epochs after it — far enough out that Earth is no longer holding it.
        let helio = |t: Epoch, st: &StateVector| {
            let (rs_km, vs_km) = eph
                .state_km_s(SUN_J2000, SSB_J2000, t.as_hifitime())
                .expect("Sun state");
            (st.position - rs_km * 1e3, st.velocity - vs_km * 1e3)
        };
        let at_ca = clock.state_at(ca.epoch).expect("state at CA");
        let (r_ca, _) = helio(ca.epoch, &at_ca);

        let t10 = ca.epoch.shifted_by_seconds(10.0 * 86_400.0);
        let seed10 = clock.state_at(t10).expect("state 10 d after CA");
        let onward = s
            .scenario
            .propagate_free(t10, seed10, 10.0 * 86_400.0, 50)
            .expect("post-encounter arc");
        let osculating = |t: Epoch| {
            let st = onward.state_at(t).expect("post-encounter state");
            let (r_h, v_h) = helio(t, &st);
            1.0 / (2.0 / r_h.norm() - v_h.norm_squared() / mu_sun)
        };
        let mut a_true = Vec::new();
        for days in SAMPLE_DAYS {
            let t = ca.epoch.shifted_by_seconds(days * 86_400.0);
            let r_geo = (onward.state_at(t).expect("state").position
                - earth.state_at(t).expect("Earth").position)
                .norm();
            a_true.push((days, osculating(t), r_geo));
        }
        // The osculating value is not the observable. It wobbles through the
        // revolution — the samples above swing ±100 km-equivalent — so the number
        // the verdict rests on is its **mean over one full post-encounter
        // revolution**, which is what kills a periodic term. The window opens at
        // `SETTLE_DAYS`, by when the rock is ~13 Hill radii out and Earth's
        // residual pull has stopped moving its energy.
        let period = std::f64::consts::TAU * (a_closed.powi(3) / mu_sun).sqrt();
        let mean_from = |start_days: f64| {
            let t0 = ca.epoch.shifted_by_seconds(start_days * 86_400.0);
            let n = MEAN_SAMPLES;
            (0..n)
                .map(|i| osculating(t0.shifted_by_seconds(period * i as f64 / n as f64)))
                .sum::<f64>()
                / n as f64
        };
        let a_mid = mean_from(SETTLE_DAYS);
        // The convention's own error bar: the same mean taken over a revolution
        // that starts half a revolution later. If the two disagree, the window is
        // not a revolution or the arc is not settled, and nothing here is safe.
        let a_shifted = mean_from(SETTLE_DAYS + period / 86_400.0 / 2.0);
        let spread_km = (a_mid - a_shifted).abs() / grad_n.abs() / 1e3;
        // A second convention check, on the other axis: opening the window later
        // still. The half-revolution shift asks whether the window is a
        // revolution; this asks whether it opened late enough for Earth to have
        // let go. A row where the two disagree has not settled.
        let a_late = mean_from(SETTLE_LATE_DAYS);

        // The r ≈ R⊕ column: the same circle with vis-viva evaluated at the rock's
        // own heliocentric distance instead of Earth's.
        let mut f_r = s.frame;
        f_r.r_earth = r_ca;
        let d_r = f_r.resonant_circle(resonance).map(|c| c.signed_distance(p));

        log(&format!(
            "\nlead {lead_days:.0} d, Δv {dv:.10} m/s, CA {}",
            ca.epoch.as_hifitime()
        ));
        log(&format!(
            "  flown point ξ {:+.1} km ζ {:+.1} km, b {:.1} km; d₀ {:+.3} km against the recorded {:+.3} km",
            p.x / 1e3,
            p.y / 1e3,
            enc.impact_parameter / 1e3,
            d0 / 1e3,
            recorded / 1e3
        ));
        log(&format!(
            "  ∇a'·n̂ {grad_n:.4} m/m; identity (a'_closed − a_res)/∇a'·n̂ {identity_km:+.3} km against d₀ {:+.3} km",
            d0 / 1e3
        ));
        for (days, a, r_geo) in &a_true {
            log(&format!(
                "  a_true(CA+{days:>3.0} d, {:5.2} Hill) {:.9} AU → (a_true − a_res)/∇a'·n̂ {:+10.1} km, (a_true − a'_closed)/∇a'·n̂ {:+10.1} km",
                r_geo / EARTH_HILL_RADIUS_M,
                a / AU_M,
                (a - a_res) / grad_n / 1e3,
                (a - a_closed) / grad_n / 1e3
            ));
        }
        log(&format!(
            "  a_true over one revolution ({:.1} d) {:.9} AU → (a_true − a_res)/∇a'·n̂ {:+.1} km, (a_true − a'_closed)/∇a'·n̂ {:+.1} km",
            period / 86_400.0,
            a_mid / AU_M,
            (a_mid - a_res) / grad_n / 1e3,
            (a_mid - a_closed) / grad_n / 1e3
        ));
        log(&format!(
            "  a'_closed {:.9} AU; the mean's own error bar (window shifted half a revolution) {spread_km:.1} km against a recorded error of {:.1} km",
            a_closed / AU_M,
            recorded / 1e3
        ));
        log(&format!(
            "  same mean opened at CA+{SETTLE_LATE_DAYS:.0} d instead: (a_true − a_res)/∇a'·n̂ {:+.1} km (moved {:+.1} km) — if this walks toward the other leads' −15 km the row was still settling",
            (a_late - a_res) / grad_n / 1e3,
            (a_late - a_mid) / grad_n / 1e3
        ));
        // Can the map get that radial offset without flying? The b-vector is the
        // rock's displacement from Earth in the b-plane, so `r⊕ + unproject(p)` is
        // the offset a drawn circle could correct for on its own.
        let r_map = s.frame.r_earth + s.frame.unproject(p);
        log(&format!(
            "\n  radial offset: flown {:+.1} km, map's own r⊕ + B̂ estimate {:+.1} km (miss {:+.1} km)",
            (r_ca.norm() - s.frame.r_earth.norm()) / 1e3,
            (r_map.norm() - s.frame.r_earth.norm()) / 1e3,
            (r_map.norm() - r_ca.norm()) / 1e3
        ));
        match d_r {
            Some(d) => log(&format!(
                "  r ≈ R⊕ column: rock at {:.6} AU vs Earth at {:.6} AU; circle moves d₀ {:+.3} → {:+.3} km (shift {:+.1} km against the {:+.1} km the truth needs)",
                r_ca.norm() / AU_M,
                s.frame.r_earth.norm() / AU_M,
                d0 / 1e3,
                d / 1e3,
                (d - d0) / 1e3,
                (a_mid - a_closed) / grad_n / 1e3
            )),
            None => log("  r ≈ R⊕ column: the substituted frame cannot reach this resonance"),
        }
        // ---- The four ingredients the closed form takes from the NOMINAL rock ----
        //
        // The map builds `Ŝ`, `v∞`, `c`, `θ` and Earth's state from the
        // **nominal**, undeflected encounter and then places the deflected point
        // on it. Six sessions have tested that as a whole — rebuild every
        // ingredient at once ("the own frame") and see whether the answer
        // improves. It does not, and non-monotonically in the lead, which is the
        // signature of terms that partially cancel; a compound test cannot see
        // them. This swaps them **one at a time**, on the same flight, with no
        // extra propagation.
        //
        // Two candidates the previous session left open — the solar tide across
        // the flyby, and the finite time the turn actually takes — are excluded
        // before this runs, by a number already on record rather than by a new
        // flight. The 300 d and the 12 yr doors sit **0.48° of arc apart on the
        // same circle**: they share `c`, share `b` to under 1 %, and share `v∞`
        // to 4e-4, i.e. they are the *same encounter* — and their biases differ
        // 20×. No encounter-local mechanism can do that. Whatever varies with the
        // lead varies *before* the rock arrives, and the incoming heliocentric
        // orbit is exactly what a Δv of 0.2165 at 12 yr and 2.880 at 300 d differ
        // in — which is what these four ingredients describe.
        //
        // Each frame is rebuilt through `OpikFrame::new` rather than by poking
        // fields, so everything a swapped ingredient determines — the axes, `θ`,
        // `c` — follows it. A frame carrying a new `v_earth` beside a stale `ζ̂`
        // is not a frame. And each re-*projects* the b-vector: the b-vector is the
        // physical object and `(ξ, ζ)` are coordinates in whichever frame is being
        // asked. (`capture_radius` is left at the nominal value when `v∞` is
        // swapped; nothing on this path reads it.)
        let (r1_km, v1_km) = eph
            .state_km_s(EARTH_J2000, SUN_J2000, ca.epoch.as_hifitime())
            .expect("Earth at this flight's own CA");
        let (r1, v1) = (r1_km * 1e3, v1_km * 1e3);
        let predict = |s_hat: Vector3<f64>, v_inf: f64, r_e: Vector3<f64>, v_e: Vector3<f64>| {
            let mut e = s.nominal;
            e.s_hat = s_hat;
            e.v_inf = v_inf;
            OpikFrame::new(&e, r_e, v_e, mu_sun)
                .ok()
                .map(|f| f.post_encounter_semi_major_axis(f.project(&enc.b_vector)))
        };
        // Every delta below is converted with the **same** nominal `∇a'·n̂` as the
        // bias it is being compared against, so the terms and the bias share one
        // unit conversion and can legitimately be summed.
        let km_eq = |a: Option<f64>| a.map_or(f64::NAN, |a| (a - a_closed) / grad_n / 1e3);
        let sn = s.frame.eta_hat;
        let un = s.frame.v_inf;
        let rn = s.frame.r_earth;
        let vn = s.frame.v_earth;
        let d_shat = km_eq(predict(enc.s_hat, un, rn, vn));
        let d_vinf = km_eq(predict(sn, enc.v_inf, rn, vn));
        let d_rearth = km_eq(predict(sn, un, r1, vn));
        let d_vearth = km_eq(predict(sn, un, rn, v1));
        let d_all = km_eq(predict(enc.s_hat, enc.v_inf, r1, v1));
        let sum = d_shat + d_vinf + d_rearth + d_vearth;
        let bias_km = (a_mid - a_closed) / grad_n / 1e3;
        // The part of the `Ŝ` term that is not a rotation at all: the flown
        // b-vector is perpendicular to its *own* asymptote, not to the nominal
        // `η̂`, so `project()` silently drops a component and the closed form is
        // evaluated at a shortened radius. Restoring `|b|` along the same in-plane
        // direction isolates how much of `d_shat` is only that.
        let out_of_plane = enc.b_vector.dot(&sn);
        let p_full = p * (enc.impact_parameter / p.norm());
        let d_bmag = (s.frame.post_encounter_semi_major_axis(p_full) - a_closed) / grad_n / 1e3;
        log("\n  --- the nominal frame's four ingredients, swapped one at a time ---");
        log(&format!("  the bias to explain (a_true − a'_closed)/∇a'·n̂ = {bias_km:+.1} km, at ∇a'·n̂ = {grad_n:.4} m/m"));
        log(&format!("  Ŝ (incoming asymptote) alone {d_shat:+10.1} km — of which {d_bmag:+.1} km is only the |b| the nominal projection drops ({:+.1} km of b lies out of the nominal plane)", out_of_plane / 1e3));
        log(&format!(
            "  v∞ alone                     {d_vinf:+10.1} km — δv∞ {:+.4} m/s ({:+.2e} rel)",
            enc.v_inf - un,
            (enc.v_inf - un) / un
        ));
        log(&format!("  Earth's position alone       {d_rearth:+10.1} km — Earth moved {:.0} km since the nominal CA", (r1 - rn).norm() / 1e3));
        log(&format!("  Earth's velocity alone       {d_vearth:+10.1} km — |δV⊕| {:.4} m/s, turned {:.3e} rad", (v1 - vn).norm(), (v1.normalize().dot(&vn.normalize())).clamp(-1.0, 1.0).acos()));
        log(&format!("  sum of the four {sum:+.1} km | all four at once (the own frame) {d_all:+.1} km | the bias {bias_km:+.1} km"));
        let linear = (sum - d_all).abs() <= 0.10 * d_all.abs().max(1.0);
        let owned = (d_all - bias_km).abs() <= 0.25 * bias_km.abs().max(1.0);
        log(&format!(
            "  → linearity {} | these four {} the bias",
            if linear {
                "HOLDS: the sum is the compound, so each term stands alone"
            } else {
                "FAILS: the terms cross, so no single one owns the answer"
            },
            if owned {
                "ACCOUNT FOR"
            } else {
                "do NOT account for"
            }
        ));
        split.push((
            lead_days,
            grad_n,
            bias_km,
            d_shat,
            d_vinf,
            d_rearth,
            d_vearth,
            d_all,
            out_of_plane / 1e3,
            d_bmag,
        ));
        // ---- The same question asked on the leg BEFORE the encounter ----
        //
        // The four ingredients above are all *inputs* to the construction, and
        // none of them is the ladder. What is left is the construction itself,
        // and it makes the same claim twice: `incoming_semi_major_axis` is the
        // identical arithmetic — `V⊕ + v∞·Ŝ`, vis-viva at Earth's distance — with
        // the incoming asymptote in place of the outgoing one. The module doc
        // already records that round-trip as good only to `8.7e-5`, which at this
        // gradient is hundreds of kilometres, i.e. the size of the residual the
        // four ingredients leave behind. So: is the error in the **turn**, or in
        // the **baseline** the turn is applied to?
        //
        // Read in *both* frames on purpose. The nominal frame's incoming `a` is
        // one number for all four flights (it does not depend on the lead), so a
        // spread against it would be `a_in_true` moving and nothing else. The
        // flight's own frame is the cell that discriminates.
        //
        // The observable mirrors the outgoing one exactly: the window **closes**
        // at CA − `SETTLE_DAYS` and runs one full revolution backward, the same
        // `MEAN_SAMPLES` points, with the same half-revolution shift as its error
        // bar. Mirrored so that `a_out − a_in` is a difference of two means taken
        // the same way and the convention cancels out of it.
        //
        // For the short-lead flights that backward arc runs past the impulse
        // epoch, onto a continuation the rock never flew. That is deliberate and
        // it is the right observable: what is wanted is *the orbit the rock is on
        // as it arrives*, which is a property of its state at CA − 30 d, not of
        // how it got there.
        let f_own = OpikFrame::new(&enc, r1, v1, mu_sun).expect("own frame");
        let a_own = f_own.post_encounter_semi_major_axis(f_own.project(&enc.b_vector));
        let a_in_nom = s.frame.incoming_semi_major_axis();
        let a_in_own = f_own.incoming_semi_major_axis();
        let t_in = ca.epoch.shifted_by_seconds(-SETTLE_DAYS * 86_400.0);
        let seed_in = clock.state_at(t_in).expect("state before CA");
        let back = s
            .scenario
            .propagate_free(t_in, seed_in, -10.0 * 86_400.0, 50)
            .expect("pre-encounter arc");
        let osc_in = |t: Epoch| {
            let st = back.state_at(t).expect("pre-encounter state");
            let (r_h, v_h) = helio(t, &st);
            1.0 / (2.0 / r_h.norm() - v_h.norm_squared() / mu_sun)
        };
        // The period comes from the *incoming* orbit, not the outgoing one — a
        // window sized by the wrong revolution is not a revolution.
        let period_in = std::f64::consts::TAU * (a_in_own.powi(3) / mu_sun).sqrt();
        let mean_in_closing = |close_days: f64| {
            let t_close = ca.epoch.shifted_by_seconds(-close_days * 86_400.0);
            let n = MEAN_SAMPLES;
            (0..n)
                .map(|i| osc_in(t_close.shifted_by_seconds(-period_in * i as f64 / n as f64)))
                .sum::<f64>()
                / n as f64
        };
        let a_in_mid = mean_in_closing(SETTLE_DAYS);
        let a_in_shifted = mean_in_closing(SETTLE_DAYS + period_in / 86_400.0 / 2.0);
        let in_spread_km = (a_in_mid - a_in_shifted).abs() / grad_n.abs() / 1e3;
        let r_geo_in = (back.state_at(t_in).expect("state").position
            - earth.state_at(t_in).expect("Earth").position)
            .norm();
        // The four cells, all in the same km-equivalent as everything above.
        let in_err_nom = (a_in_mid - a_in_nom) / grad_n / 1e3;
        let in_err_own = (a_in_mid - a_in_own) / grad_n / 1e3;
        let out_err_nom = (a_mid - a_closed) / grad_n / 1e3;
        let out_err_own = (a_mid - a_own) / grad_n / 1e3;
        // The sharper form: does the construction get the *change* right even
        // where it gets both absolutes wrong? If it does, a circle should be
        // drawn on `Δa`, not on `a'`.
        let d_true = (a_mid - a_in_mid) / grad_n / 1e3;
        let d_pred_nom = (a_closed - a_in_nom) / grad_n / 1e3;
        let d_pred_own = (a_own - a_in_own) / grad_n / 1e3;
        log("\n  --- the same construction, asked on the incoming leg (the 2x2) ---");
        log(&format!("  incoming window closes at CA-{SETTLE_DAYS:.0} d ({:.2} Hill), one revolution of {:.1} d backward, {MEAN_SAMPLES} samples; its own error bar (window closed half a revolution earlier) {in_spread_km:.1} km", r_geo_in / EARTH_HILL_RADIUS_M, period_in / 86_400.0));
        log(&format!("  a_in_true {:.9} AU | a_in nominal frame {:.9} AU (err {in_err_nom:+.1} km) | a_in own frame {:.9} AU (err {in_err_own:+.1} km)", a_in_mid / AU_M, a_in_nom / AU_M, a_in_own / AU_M));
        log(&format!("  a_out_true {:.9} AU | a' nominal frame (err {out_err_nom:+.1} km) | a' own frame (err {out_err_own:+.1} km)", a_mid / AU_M));
        log(&format!("  the CHANGE across the encounter: true {d_true:+.1} km | predicted, nominal frame {d_pred_nom:+.1} km (out by {:+.1}) | predicted, own frame {d_pred_own:+.1} km (out by {:+.1})", d_pred_nom - d_true, d_pred_own - d_true));
        // Where the *own* frame's baseline error comes from. `incoming_semi_major_axis`
        // evaluates vis-viva at **Earth's** heliocentric distance while the rock is
        // tens of thousands of kilometres away; the incoming speed does not depend on
        // `r` at all, so re-evaluating at the rock's own distance is exact arithmetic
        // rather than a re-solve. This is the `r ~ R_earth` substitution the module
        // doc has blamed since it was written, asked on the leg it actually applies to.
        let v2_in_own = (f_own.v_earth + f_own.v_inf * f_own.eta_hat).norm_squared();
        let a_in_at_rock = 1.0 / (2.0 / r_ca.norm() - v2_in_own / mu_sun);
        let radial_km = (a_in_at_rock - a_in_own) / grad_n / 1e3;
        log(&format!("  of that own-frame baseline error, the r ~ R_earth substitution accounts for {radial_km:+.1} km (rock {:+.1} km from Earth's distance), leaving {:+.1} km against a bar of {in_spread_km:.0} km", (r_ca.norm() - f_own.r_earth.norm()) / 1e3, in_err_own - radial_km));
        let baseline = (in_err_own - out_err_own).abs() <= 0.25 * out_err_own.abs().max(1.0);
        log(&format!("  -> in the flight's own frame the incoming error is {in_err_own:+.1} km against an outgoing {out_err_own:+.1} km: {}", if baseline { "THE SAME — a baseline offset the turn carries through, so the circle belongs on the change in a" } else { "NOT the same — the baseline is not what is wrong, and the turn owns the residual" }));
        inbound.push((
            lead_days,
            p.x / 1e3,
            in_err_nom,
            in_err_own,
            out_err_nom,
            out_err_own,
            d_true,
            d_pred_nom,
            d_pred_own,
            in_spread_km,
        ));
        // What the map would draw if the circle were placed on the **change** in
        // `a` instead of on the absolute `a'`: the locus where the closed form's
        // `a' − a_in` equals `a_res − a_in_true`, with `a_in_true` the flight's
        // own measured incoming orbit. Along the normal that is a pure shift of
        // the circle by `(a_in_true − a_in_closed)/∇a'`, so it needs no re-solve.
        //
        // Two versions, because what ships has to be affordable: the 32-sample
        // revolution mean (one extra backward integration of ~288 d), and the
        // single osculating sample at the window's own closing epoch (free — the
        // planner already holds that state). If the cheap one is as flat, the
        // repair costs nothing.
        let a_in_one = osc_in(t_in);
        let d0_km = d0 / 1e3;
        let d0_mean = d0_km + (a_in_mid - a_in_nom) / grad_n / 1e3;
        let d0_one = d0_km + (a_in_one - a_in_nom) / grad_n / 1e3;
        log(&format!("  if the circle were drawn on the CHANGE in a: door at {d0_mean:+.1} km (revolution mean) or {d0_one:+.1} km (one sample at CA-{SETTLE_DAYS:.0} d), against {d0_km:+.1} km as it ships"));
        repair.push((lead_days, d0_km, d0_mean, d0_one, in_spread_km));
        // Which heliocentric distance *would* have been right? The outgoing speed
        // does not depend on `r` at all — `cos θ'` is a b-plane quantity — so `a'`
        // depends on it only through vis-viva, and the `r` that reproduces the
        // flown orbit inverts in closed form rather than being searched for.
        let v2_out = {
            let v = s.frame.v_earth.norm();
            let u = s.frame.v_inf;
            v * v + u * u + 2.0 * v * u * s.frame.cos_theta_out(p)
        };
        let r_star = 2.0 / (1.0 / a_mid + v2_out / mu_sun);
        log(&format!(
            "  the r that would have been right: {:+.1} km from Earth's, i.e. {:.2} of the way to the rock's own {:+.1} km",
            (r_star - s.frame.r_earth.norm()) / 1e3,
            (r_star - s.frame.r_earth.norm()) / (r_ca.norm() - s.frame.r_earth.norm()),
            (r_ca.norm() - s.frame.r_earth.norm()) / 1e3
        ));
        let verdict = if spread_km >= recorded.abs() / 3e3 {
            "INCONCLUSIVE at this lead — the sampling spread is not small against the error being explained"
        } else if ((a_mid - a_res) / grad_n).abs() < 0.25 * recorded.abs() {
            "the CONDITION holds and the PREDICTION is biased — the flyby does not leave the rock where the closed form says"
        } else if ((a_mid - a_closed) / grad_n).abs() < 0.25 * recorded.abs() {
            "the PREDICTION holds and the CONDITION is wrong — a return does not need a = a_res"
        } else {
            "NEITHER: both differences are large, which the decision rule does not cover"
        };
        log(&format!("  verdict (revolution mean): {verdict}"));
    }
    if !split.is_empty() {
        log("\n=== WHICH NOMINAL INGREDIENT CARRIES THE PREDICTION BIAS? ===\n");
        log("Every column is km-equivalent on the b-plane, converted with that row's own nominal ∇a'·n̂.");
        log("`bias` is what the flown orbit says the prediction is out by; each ingredient column is how far swapping that one ingredient alone moves the prediction. A column that reproduces `bias` is the mechanism.\n");
        log("| lead | ∇a'·n̂ (m/m) | bias | Ŝ alone | v∞ alone | r⊕ alone | V⊕ alone | sum | all four | b out of plane | of which b-magnitude |");
        log("|---|---|---|---|---|---|---|---|---|---|---|");
        for (lead, g, bias, ds, dv, dr, dvv, dall, oop, dbm) in &split {
            log(&format!("| {lead:.0} d | {g:.4} | **{bias:+.1} km** | {ds:+.1} | {dv:+.1} | {dr:+.1} | {dvv:+.1} | {:+.1} | {dall:+.1} | {oop:+.1} km | {dbm:+.1} |", ds + dv + dr + dvv));
        }
        // The gradient is the closed form's own, and it scales every km-equivalent
        // in the ladder this campaign exists to explain. If it moved materially
        // across the leads, part of the 19 → 786 km ladder would be a unit
        // conversion rather than an `a'` error, and nothing below could be read.
        let gmin = split.iter().map(|r| r.1).fold(f64::INFINITY, f64::min);
        let gmax = split.iter().map(|r| r.1).fold(f64::NEG_INFINITY, f64::max);
        log(&format!("\n∇a'·n̂ across these leads: {gmin:.4} to {gmax:.4} m/m, a spread of {:.2}%. The km-equivalents above are commensurable to that much, and no more.", 100.0 * (gmax - gmin) / gmax.abs()));
    }
    if !inbound.is_empty() {
        log("\n=== IS IT THE TURN, OR THE BASELINE THE TURN IS APPLIED TO? ===\n");
        log("The same construction asked on both legs of the same flight. Columns are km-equivalent at that row's nominal ∇a'·n̂. `err` is measured minus predicted, so a negative number means the construction says the orbit is bigger than it is.\n");
        log("| lead | ξ of the flown point | a_in err, nominal frame | a_in err, own frame | a_out err, nominal frame | a_out err, own frame | error bar on a_in |");
        log("|---|---|---|---|---|---|---|");
        for (lead, xi, en, eo, on_, oo, _, _, _, bar) in &inbound {
            log(&format!("| {lead:.0} d | {xi:+.0} km | {en:+.1} | **{eo:+.1}** | {on_:+.1} | **{oo:+.1}** | ±{bar:.1} |"));
        }
        log("\nAnd the same rows as a change across the encounter, which is what a resonant circle actually needs to get right:\n");
        log("| lead | true Δa | predicted Δa, nominal frame | out by | predicted Δa, own frame | out by |");
        log("|---|---|---|---|---|---|");
        for (lead, _, _, _, _, _, dt, dn, dw, _) in &inbound {
            log(&format!(
                "| {lead:.0} d | {dt:+.1} km | {dn:+.1} km | {:+.1} | {dw:+.1} km | {:+.1} |",
                dn - dt,
                dw - dt
            ));
        }
    }
    if !repair.is_empty() {
        log("\n=== WHAT THE REPAIR WOULD BUY: THE CIRCLE DRAWN ON THE CHANGE IN a ===\n");
        log("`as shipped` is the door's distance from the circle the map draws today. The other two place the same circle at the locus where the closed form's a' − a_in equals a_res − a_in_true, using the flight's own incoming orbit as the baseline.\n");
        log("| lead | as shipped | on the change (revolution mean) | on the change (one sample) | error bar on the baseline |");
        log("|---|---|---|---|---|");
        for (lead, d0k, dm, do1, bar) in &repair {
            log(&format!(
                "| {lead:.0} d | {d0k:+.1} km | **{dm:+.1} km** | {do1:+.1} km | ±{bar:.1} km |"
            ));
        }
        let span = |f: fn(&RepairRow) -> f64| {
            let lo = repair.iter().map(f).fold(f64::INFINITY, f64::min);
            let hi = repair.iter().map(f).fold(f64::NEG_INFINITY, f64::max);
            (lo, hi, hi - lo)
        };
        let (s_lo, s_hi, s_sp) = span(|r| r.1);
        let (m_lo, m_hi, m_sp) = span(|r| r.2);
        let (o_lo, o_hi, o_sp) = span(|r| r.3);
        log(&format!(
            "\nas shipped: {s_lo:+.1} to {s_hi:+.1} km, a ladder {s_sp:.1} km wide."
        ));
        log(&format!(
            "on the revolution-mean change: {m_lo:+.1} to {m_hi:+.1} km, spread {m_sp:.1} km."
        ));
        log(&format!(
            "on one osculating sample: {o_lo:+.1} to {o_hi:+.1} km, spread {o_sp:.1} km."
        ));
        log("The number a band has to cover is the spread, not the offset: a constant offset is a circle drawn in the wrong place by a fixed amount, which is a correction; a ladder is not.");
    }
    log(&format!("\nstage took {:.1} s", t0.elapsed().as_secs_f64()));
}
