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

/// The Δv span the ladder covers, m/s. The low end is below any reachable
/// resonance; the high end is where the deflected pass starts leaving the
/// shipping 5e8 m scan gate, which the ladder reports rather than assumes.
const LADDER_DV_LO: f64 = 0.004;
const LADDER_DV_HI: f64 = 0.80;
const LADDER_RUNGS: usize = 14;

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
    fn load(path: &std::path::Path) -> Option<Self> {
        let text = std::fs::read_to_string(path).ok()?;
        let mut rungs = Vec::new();
        for line in text.lines().skip(1) {
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
    epoch0: asteroid_core::Epoch,
    prograde: Vector3<f64>,
    /// The ξ every aim holds: the nominal encounter's own, which a small
    /// along-track nudge barely moves. Same convention as `probe_keyhole_return`,
    /// so the numbers here are commensurable with the published 3:4 ones.
    xi: f64,
}

fn setup() -> Setup {
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
    let epoch0 = scenario.epoch0();
    let seed = ds.nominal().state_at(epoch0).expect("seed");
    let prograde = along_track_unit(seed).expect("along-track");
    let xi = frame.project(&nominal.b_vector).x;
    drop(ds);
    Setup {
        scenario,
        nominal,
        frame,
        epoch0,
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
        other => {
            eprintln!("unknown stage {other:?}; expected ladder | screen | door | spacing");
            std::process::exit(2);
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 1 — the b(Δv) curve, sampled once
// ---------------------------------------------------------------------------

fn stage_ladder(args: &[String]) {
    let rungs: usize = args
        .get(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(LADDER_RUNGS);
    let s = setup();
    let ds = s.scenario.deflection().expect("deflection");
    let dir = work_dir();
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("keyhole_ladder.tsv");
    let mut out = String::from("dv_signed_m_s\tb_m\txi_m\tzeta_m\n");

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
    for sign in [-1.0f64, 1.0] {
        for i in 0..rungs {
            let f = i as f64 / (rungs - 1) as f64;
            let dv = LADDER_DV_LO * (LADDER_DV_HI / LADDER_DV_LO).powf(f);
            let delta = s.prograde * (sign * dv);
            flights += 1;
            match ds.evaluate(s.epoch0, delta) {
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
    if let Some(l) = Ladder::load(&path) {
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
    let max_years: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(7);
    let s = setup();
    let circles = s.frame.resonant_circles(
        2..=max_years.max(2),
        MAX_REVOLUTIONS,
        B_MAX_CAPTURE_RADII * s.nominal.capture_radius,
    );
    let mut bs: Vec<(f64, String)> = Vec::new();
    for c in &circles {
        for branch in [CircleBranch::Minus, CircleBranch::Plus] {
            let Ok(aim) = aim_at_resonance(&s.frame, c.resonance, s.xi, branch) else {
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
        "\n{} reachable circles at ξ = {:.0} km. Nearest-neighbour gaps in b:\n",
        bs.len(),
        s.xi / 1e3
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
    let max_flights: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(14);
    let Some(ladder) = Ladder::load(&work_dir().join("keyhole_ladder.tsv")) else {
        eprintln!("no keyhole_ladder.tsv in the work dir — run the `ladder` stage first");
        std::process::exit(3);
    };
    let s = setup();
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
        deflection_epoch: s.epoch0,
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
    let h: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(3);
    let k: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4);
    let branch = match args.get(3).map(|s| s.to_lowercase()) {
        Some(s) if s.starts_with('p') => CircleBranch::Plus,
        _ => CircleBranch::Minus,
    };
    let iterations: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(20);
    let resonance = Resonance { h, k };

    let Some(ladder) = Ladder::load(&work_dir().join("keyhole_ladder.tsv")) else {
        eprintln!("no keyhole_ladder.tsv in the work dir — run the `ladder` stage first");
        std::process::exit(3);
    };
    let s = setup();
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
        deflection_epoch: s.epoch0,
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
