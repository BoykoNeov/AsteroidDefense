//! Does the ±3σ shell still support the **shape** the b-plane view draws?
//!
//! `uncertainty.rs` has had the linearity shell since Tier 3 began, and
//! `RealFieldScenario::bplane_uncertainty_checked` has always returned it. What has
//! never been done is point it at the *drawn* ellipse — the 168.71 × 0.82 km needle
//! the Godot view puts on screen on `[U]`, at every setting of the `[Z]`/`[X]` σ
//! knob. That gap is roadmap item 8's second half.
//!
//! **Why the shipping scalar cannot answer it.** [`LinearityReport`] normalises the
//! worst residual against `shell_scale`, the largest flown displacement anywhere on
//! the shell. On a 205:1 ellipse that scale is a *length* — 0.693 of the 3σ
//! half-length here, since the shell points are the state covariance's principal
//! axes and their images do not line up with this ellipse's — so a residual of a
//! few kilometres — enough to be several times the 0.82 km minor axis and to turn
//! the needle into something with real width — divides down to a per-cent number
//! and reads as "linear". The scalar is right for what it was built for (the
//! probability, which the major axis dominates) and blind to the shape.
//!
//! So this probe decomposes the same residuals **along the drawn ellipse's own
//! axes**, through `LinearityReport::shape_residual` — which lived here as an
//! inline eigen block when it was first measured, and moved into the module once
//! `core/tests/tier3_drawn_shape.rs` turned out to hold a second copy of it:
//!
//!   * `major` — residual projected on the major axis, against `n_sigma · σ_major`.
//!     The number the existing scalar approximates.
//!   * `minor` — residual projected on the minor axis, against `n_sigma · σ_minor`.
//!     The number nobody has ever read. If it exceeds 1, the drawn width of the
//!     needle is smaller than the error in knowing where the needle is, and the
//!     view is drawing a precision it does not have.
//!
//! **On the frame.** The instinct is that this decomposition has to happen in the
//! display frame, for the reason `Tier3View` was built around — an ellipse's
//! orientation is not invariant under the sensitivity's arbitrary-but-deterministic
//! basis choice. It turns out the two ratios do not need it: the residual and the
//! axis are expressed in the *same* basis, and a common rotation leaves their dot
//! product alone, so `of major` and `of minor` are invariant scalars after all.
//! What genuinely needs the rotation is the `deg` column — the angle the view
//! prints — and that is why the rotation is done here anyway, on the same call
//! `Tier3View::build` makes, which also re-measures the two b-planes' agreement.
//!
//! **What it sweeps.** The σ knob, over its full range — `10^±3` about the shipping
//! covariance — because linearity at 1σ of a 1000× covariance is a different
//! question from linearity at 1σ of the shipping one, and the answer worth printing
//! is the scale at which the drawn shape stops being supported, not three sample
//! points either side of it.
//!
//! Requires kernels. ~13 + 12·n propagations; the default 7-point sweep is ~97,
//! about 4 minutes.
//!
//! Arguments are σ-knob scales; `rtol=<x>` sets the forward integration tolerance
//! instead of the shipping `1e-9`, which is how the residual floor below is told
//! apart from curvature — a floor made of integration noise moves when the
//! tolerance does, and real curvature does not.
//!
//!   cargo run -p asteroid_core --release --example probe_tier3_drawn_shape -- [scale ...] [rtol=1e-13]

use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use asteroid_core::{ImpactorConfig, OpikFrame, RealFieldScenario, StateCovariance};
use std::time::Instant;

/// The three covariance constants the frontend draws with, mirrored from
/// `godot/rust/src/mission_core.rs` (`TIER3_ALONG_TRACK_SIGMA_MS`,
/// `TIER3_VELOCITY_ANISOTROPY`, `TIER3_POSITION_SIGMA_M`) as the other probes
/// mirror the scan gate. `probe_keyhole_map` holds a third copy inline. Nothing
/// enforces the three agreeing — core cannot see the binding — so a change to the
/// drawn covariance has to be made in all three places or this probe silently
/// answers about an ellipse nobody draws.
const ALONG_TRACK_SIGMA_MS: f64 = 5.0e-5;
const VELOCITY_ANISOTROPY: f64 = 20.0;
const POSITION_SIGMA_M: f64 = 1.0e3;
/// The `[Z]`/`[X]` knob's stops: `10^±TIER3_SCALE_DECADES`.
const SCALE_DECADES: f64 = 3.0;
/// How far out the shell is flown. Three σ is the convention `uncertainty.rs`
/// argues for and the number roadmap item 8 names.
const N_SIGMA: f64 = 3.0;

/// Metres per kilometre — this probe reports in km at the print boundary, as the
/// other map probes do.
const M_PER_KM: f64 = 1.0e3;

/// The fraction of an axis the residual may eat before the drawn extent along that
/// axis is a claim the linearisation does not support. Not a physical constant —
/// a reading threshold, stated so the verdict below can be argued with.
const SHAPE_TOLERANCE: f64 = 0.25;

fn main() {
    let mut scales: Vec<f64> = Vec::new();
    let mut rtol: Option<f64> = None;
    for a in std::env::args().skip(1) {
        match a.strip_prefix("rtol=") {
            Some(v) => rtol = Some(v.parse().expect("rtol must be a number")),
            None => scales.push(a.parse().expect("scale must be a number")),
        }
    }
    if scales.is_empty() {
        scales = vec![1.0e-3, 1.0e-2, 1.0e-1, 1.0, 1.0e1, 1.0e2, 1.0e3];
    }

    let mut cfg = ImpactorConfig::default();
    if let Some(r) = rtol {
        cfg.forward_rtol = r;
    }
    println!("forward rtol: {:.0e}", cfg.forward_rtol);
    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&cfg) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e}");
            eprintln!("(this probe needs DE440 kernels — see kernels::resolve)");
            std::process::exit(1);
        }
    };
    println!("build: {:.2} s", t.elapsed().as_secs_f64());

    // The seed the covariance describes, and the display frame the view draws in.
    // Both come from this scenario so the ellipse below is the same object the
    // frontend puts on screen, not a lookalike built from the same constants.
    let ds = scenario.deflection().expect("deflection");
    let seed = ds
        .nominal()
        .state_at(scenario.epoch0())
        .expect("seed at epoch0");

    let covariances: Vec<StateCovariance> = scales
        .iter()
        .map(|s| {
            StateCovariance::synthetic_along_track(
                seed,
                ALONG_TRACK_SIGMA_MS * s,
                VELOCITY_ANISOTROPY,
                POSITION_SIGMA_M * s,
            )
            .expect("non-degenerate seed")
        })
        .collect();

    let knob_lo = 10.0_f64.powf(-SCALE_DECADES);
    let knob_hi = 10.0_f64.powf(SCALE_DECADES);
    println!(
        "sweeping {} scales, shell at {N_SIGMA:.0}σ, knob range {knob_lo:.0e}…{knob_hi:.0e}",
        scales.len()
    );

    let t = Instant::now();
    let (sens, checked) = scenario
        .bplane_uncertainty_checked_many(&covariances, N_SIGMA)
        .expect("sensitivity and shells");
    println!(
        "{} propagations: {:.1} s",
        13 + 12 * covariances.len(),
        t.elapsed().as_secs_f64()
    );

    // The rotation onto the view's axes — the same call `Tier3View::build` makes,
    // built from the sensitivity's own reduction so the two b-planes share Ŝ.
    let eph = scenario.ephemeris().clone();
    let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
    let t_ca = ds
        .nominal_encounter_epoch()
        .expect("encounter epoch")
        .expect("a close approach inside the scan gate");
    let (r_e_km, v_e_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
        .expect("Earth heliocentric state at encounter");
    let frame =
        OpikFrame::new(&sens.nominal, r_e_km * 1e3, v_e_km * 1e3, mu_sun).expect("Öpik frame");
    let (rot, orth) = sens.basis.rotation_to(frame.xi_hat, frame.zeta_hat);
    if orth > 1e-9 {
        eprintln!("*** basis change is not orthonormal ({orth:.3e}): the two frames share no Ŝ");
        std::process::exit(1);
    }
    println!("display rotation residual ‖RRᵀ−I‖∞ = {orth:.3e}");

    println!();
    println!(
        "{:>8}  {:>10}  {:>9}  {:>7}  {:>10}  {:>10}  {:>9}  {:>7}",
        "scale", "σ_maj km", "σ_min km", "deg", "resid km", "of major", "of minor", "scalar"
    );

    let mut worst_minor = 0.0_f64;
    let mut rows: Vec<(f64, f64, f64)> = Vec::new(); // scale, |residual| km, of_minor
    for (scale, (unc, report)) in scales.iter().zip(checked.iter()) {
        // The decomposition itself is the core's, and needs no rotation: the
        // residual and the axis live in the same basis, so their dot product is
        // invariant. What the rotation is for is the printed *angle* — the axis
        // carried into the frame the view actually draws in.
        let shape = report
            .shape_residual(unc)
            .expect("the drawn ellipse is not degenerate");
        let (sig_major, sig_minor) = (shape.sigma_major, shape.sigma_minor);
        // The axis has no direction, and the rotation can land it on either side, so
        // fold the angle into a half-turn before printing it. Without this the
        // published map's 89.736 comes back as -90.26 and reads as a different
        // ellipse. `shape.major_hat` is sign-pinned in the module; that pinning does
        // not survive an arbitrary rotation, which is the point of folding here.
        let major_xz = rot * shape.major_hat;
        let mut angle_deg = major_xz.y.atan2(major_xz.x).to_degrees();
        if angle_deg <= -90.0 {
            angle_deg += 180.0;
        } else if angle_deg > 90.0 {
            angle_deg -= 180.0;
        }
        let (of_major, of_minor) = (shape.major_ratio, shape.minor_ratio);
        worst_minor = worst_minor.max(of_minor);
        rows.push((*scale, report.max_residual / M_PER_KM, of_minor));

        println!(
            "{:>8.0e}  {:>10.3} {:>10.4}  {:>7.2}  {:>10.4}  {:>10.4}  {:>9.3}  {:>7.4}",
            scale,
            sig_major / M_PER_KM,
            sig_minor / M_PER_KM,
            angle_deg,
            report.max_residual / M_PER_KM,
            of_major,
            of_minor,
            report.max_relative_residual
        );
    }

    println!();
    println!(
        "worst minor-axis residual over the sweep: {worst_minor:.3} of the drawn \
         {N_SIGMA:.0}σ half-width"
    );
    // The residual is two things added together, and reading it as one number is
    // how a floor gets reported as curvature. Second-order curvature grows as the
    // square of the scale, so it is invisible below some scale and a hundredfold
    // per decade above it; whatever is left at the small end is the *measurement*
    // floor — integration and differencing noise, flat in the scale because it has
    // nothing to do with the covariance. Separate them by taking the flat part as
    // the smallest residual seen and fitting the square-law part at the top.
    let floor_km = rows.iter().map(|r| r.1).fold(f64::INFINITY, f64::min);
    if let Some((s_top, resid_top, of_top)) = rows.last().copied() {
        let k = (resid_top - floor_km).max(0.0) / (s_top * s_top);
        println!("residual = {floor_km:.4} km floor + {k:.3e} x scale^2 km");
        if k > 0.0 {
            println!(
                "  -> curvature overtakes the floor at scale {:.2}",
                (floor_km / k).sqrt()
            );
        }
        // The ratio to the drawn half-width is residual/scale, so in the
        // curvature-dominated regime it grows linearly. Extrapolate to the scale
        // that would make the drawn width no wider than the error in it.
        if of_top > 0.0 {
            println!(
                "  -> the minor-axis residual reaches the drawn half-width at scale {:.0e}, \
                 against a knob that stops at {:.0e}",
                s_top / of_top,
                10.0_f64.powf(SCALE_DECADES)
            );
        }
    }
    let breaks: Vec<String> = rows
        .iter()
        .filter(|r| r.2 > SHAPE_TOLERANCE)
        .map(|r| format!("{:.0e}", r.0))
        .collect();
    if breaks.is_empty() {
        println!("no scale on the sweep exceeds {SHAPE_TOLERANCE:.2} of the drawn half-width");
    } else {
        println!(
            "scales over {SHAPE_TOLERANCE:.2} of the drawn half-width: {}",
            breaks.join(", ")
        );
    }
}
