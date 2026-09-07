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
//! ```
//!
//! `ladder` must run first; the other two read `keyhole_ladder.tsv` from the work
//! directory (`ASTEROID_PROBE_DIR`, default `W:/temp/claude/keyhole_placement`).
//! Every stage appends to a log there, so a long survey survives being
//! interrupted.
//!
//! Requires kernels.

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{
    aim_at_resonance, along_track_unit, fly_keyhole_shot, refine_keyhole_return, BPlaneEncounter,
    CircleBranch, ImpactorConfig, KeyholeAim, KeyholeAiming, KeyholeRefineTol, KeyholeShot,
    KeyholeShotOptions, OpikFrame, RealFieldScenario, Resonance, ResonantCircle, AU_M,
};
use nalgebra::Vector3;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;
use std::time::Instant;

/// Census bounds — the same ones `probe_keyhole_map` draws, so this probe can
/// never consider a resonance the map does not show.
const RETURN_YEARS: std::ops::RangeInclusive<u32> = 2..=20;
const MAX_REVOLUTIONS: u32 = 24;
const B_MAX_CAPTURE_RADII: f64 = 60.0;

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
        other => {
            eprintln!(
                "unknown stage {other:?}; expected ladder | screen | door | spacing | xi_sweep"
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
    let Some(aim_dv) = ladder.dv_for_b(aim.impact_parameter_m, sign) else {
        eprintln!(
            "{resonance} wants b = {:.0} km, outside the ladder's range",
            aim.impact_parameter_m / 1e3
        );
        std::process::exit(1);
    };
    let circle = aim.circle;
    let kh = s.frame.keyhole_at(&circle, aim.target);
    log(&format!(
        "{resonance} {branch:?}: a' {:.6} AU, aim (ξ {:.0}, ζ {:.0}) km, b {:.0} km, \
         perigee {:.0} km\n  linearised door at the aim point: {:.3} km wide \
         (|∇a'| {:.3e}); ladder aim Δv {:.10} m/s ({} nudge)",
        circle.a_prime / AU_M,
        aim.target.x / 1e3,
        aim.target.y / 1e3,
        aim.impact_parameter_m / 1e3,
        aim.perigee_m / 1e3,
        kh.width / 1e3,
        kh.gradient.norm(),
        aim_dv,
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
        // circle. `None` is the scan gate, which ends the sweep upward.
        let fly = |dv: f64| -> Option<(nalgebra::Vector2<f64>, f64, f64)> {
            match ds.evaluate(epoch, dir * dv) {
                Ok(Some(enc)) => {
                    let p = s.frame.project(&enc.b_vector);
                    Some((p, enc.impact_parameter, circle.signed_distance(p)))
                }
                _ => None,
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
        for i in 0..SWEEP_RUNGS {
            let f = i as f64 / (SWEEP_RUNGS - 1) as f64;
            let dv = SWEEP_DV_LO * (SWEEP_DV_HI / SWEEP_DV_LO).powf(f);
            flights += 1;
            let Some((p, _, d)) = fly(dv) else {
                gated_at = Some(dv);
                break;
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
                        "no sign change up to Δv {SWEEP_DV_HI} m/s (last distance {:.0} km)",
                        prev.map(|p| p.1 / 1e3).unwrap_or(f64::NAN)
                    ),
                }
            ));
            continue;
        };

        // --- bisect the crossing ---------------------------------------------
        flights += 1;
        let d_lo = fly(lo).map(|x| x.2).unwrap_or(f64::NAN);
        let mut best: Option<(f64, nalgebra::Vector2<f64>, f64, f64)> = None;
        for _ in 0..SWEEP_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            flights += 1;
            let Some((p, b, d)) = fly(mid) else { break };
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
        "\nagainst the five flown doors: placement errors ran 2.0 .. 26.8 km, and the \
         shipping band is KEYHOLE_PLACEMENT_KM = 100 km.\n  -> {}",
        if spread > 100.0e3 {
            "the crossings are further apart than the whole band, so position on the \
             circle is a first-order variable and the doors must be flown at each lead"
        } else if spread > 27.0e3 {
            "the crossings are further apart than the largest placement error seen, so \
             the sweep can separate position-on-circle from resonance identity — fly them"
        } else {
            "the crossings are closer together than the placement errors already \
             measured, so this sweep cannot resolve a variation along the circle; the \
             band's five-point basis is what it is"
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
