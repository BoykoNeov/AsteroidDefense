//! The guard on the *shape* the b-plane view draws — roadmap item 8, second half.
//!
//! `LinearityReport::max_relative_residual` has always answered "did the linear
//! map survive to 3σ?" for the quantity the module was built for, the impact
//! probability. It normalises the worst residual against the largest displacement
//! anywhere on the shell, which on a 205:1 ellipse is a **length** scale (0.693 of
//! the 3σ half-length, measured — the shell points are the *state* covariance's
//! principal axes, and their b-plane images do not line up with this ellipse's) — so a
//! residual several times the 0.82 km minor axis still divides down to a per-mil
//! number and reads as "linear". Every number the frontend draws that is *not* the
//! probability — the minor axis, and therefore the drawn width of the needle —
//! lives inside that blind spot.
//!
//! So this resolves the same residuals along the ellipse's **own** axes, through
//! `LinearityReport::shape_residual`. Frame-independently: both the residual and
//! the axis are expressed in the same b-plane basis, and a common rotation leaves
//! their dot product alone, so no Öpik rotation is needed here. (It *is* needed to
//! draw the ellipse — an orientation is not invariant — which is what `Tier3View`
//! does and what `probe_tier3_drawn_shape` prints.)
//!
//! Measured 2026-09-07 by that probe, over the σ knob's whole `10^±3` range:
//!
//! ```text
//!    scale    σ_maj km   σ_min km    resid km    of major   of minor   scalar
//!     1e-3       0.169     0.0008      0.0302      0.0596      0.355   0.0801
//!      1e0     168.710     0.8182      0.0121      0.0000      0.001   0.0000
//!      1e3  168710.343   818.2179   1528.4543      0.0013      0.561   0.0043
//! ```
//!
//! Two regimes, and only one of them is physics. The residual is a flat floor plus
//! a term growing as the square of the scale; re-running at `forward_rtol = 1e-13`
//! drops the floor a hundredfold (30 m → 0.3 m, and the `1e-3` row's minor ratio
//! 0.355 → 0.003) while leaving the curvature at `1e2` alone (15.50 → 15.33 km).
//! So the small-covariance end is the *integrator's* noise, not a bent map — read
//! naively it would have said the picture is least trustworthy when the orbit is
//! best known, which is backwards.
//!
//! The verdict: the drawn shape is supported everywhere the knob reaches. This
//! pins the two ends of that claim.

use asteroid_core::uncertainty::StateCovariance;
use asteroid_core::{ImpactorConfig, RealFieldScenario};

/// The covariance the frontend draws, mirrored from `TIER3_ALONG_TRACK_SIGMA_MS`,
/// `TIER3_VELOCITY_ANISOTROPY` and `TIER3_POSITION_SIGMA_M` in
/// `godot/rust/src/mission_core.rs`. Core cannot see the binding, so a change
/// there has to be made here too or this guards an ellipse nobody draws.
const ALONG_TRACK_SIGMA_MS: f64 = 5.0e-5;
const VELOCITY_ANISOTROPY: f64 = 20.0;
const POSITION_SIGMA_M: f64 = 1.0e3;
/// The top stop of the `[Z]`/`[X]` knob (`10^TIER3_SCALE_DECADES`).
const KNOB_TOP: f64 = 1.0e3;
/// The shell is flown at 3σ — the convention `uncertainty.rs` argues for.
const N_SIGMA: f64 = 3.0;

/// What the shipping setting is allowed to spend of its drawn minor half-width.
/// Measured 0.001; a factor of fifty of headroom, because this is a guard against
/// the shape quietly stopping being supported, not a pin on a physical constant.
const SHIPPING_LIMIT: f64 = 0.05;
/// What the knob's top stop is allowed to spend. Measured 0.561 — the drawn width
/// is still 1.8× the error in it there, and the extrapolated crossing is at scale
/// ~2e3, outside a knob that stops at 1e3. Below 1.0 is the claim worth guarding:
/// above it, the view would be drawing a width narrower than its own uncertainty.
const KNOB_TOP_LIMIT: f64 = 0.85;

/// The largest residual component along each principal axis of the mapped
/// ellipse, as a fraction of that axis' `n_sigma` half-width: `(major, minor)`.
///
/// This lived here as a private helper when it was first measured, and moved into
/// `LinearityReport::shape_residual` once `probe_tier3_drawn_shape` turned out to
/// hold a second copy of the same eigen block. The unit tests beside it in
/// `uncertainty.rs` pin it against a planted residual, which is a thing this
/// kernel-gated test cannot do.
fn axis_ratios(
    unc: &asteroid_core::uncertainty::BPlaneUncertainty,
    report: &asteroid_core::uncertainty::LinearityReport,
) -> (f64, f64) {
    let shape = report
        .shape_residual(unc)
        .expect("the drawn ellipse is not degenerate");
    (shape.major_ratio, shape.minor_ratio)
}

/// ~37 propagations (one sensitivity, two shells), about a minute.
#[test]
fn the_drawn_ellipse_shape_survives_the_shell_at_both_ends_of_the_sigma_knob() {
    if asteroid_core::kernels::resolve_for_test("Tier-3 drawn-shape linearity guard").is_none() {
        return;
    }
    let scenario = RealFieldScenario::build(&ImpactorConfig::default()).expect("build scenario");
    let seed = scenario
        .deflection()
        .expect("deflection")
        .nominal()
        .state_at(scenario.epoch0())
        .expect("seed at epoch0");

    let covariances: Vec<StateCovariance> = [1.0, KNOB_TOP]
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

    let (_, checked) = scenario
        .bplane_uncertainty_checked_many(&covariances, N_SIGMA)
        .expect("sensitivity and shells");

    let (maj_ship, min_ship) = axis_ratios(&checked[0].0, &checked[0].1);
    let (maj_top, min_top) = axis_ratios(&checked[1].0, &checked[1].1);
    println!(
        "shipping: major {maj_ship:.4}, minor {min_ship:.4} (scalar {:.4})\n\
         knob top: major {maj_top:.4}, minor {min_top:.4} (scalar {:.4})",
        checked[0].1.max_relative_residual, checked[1].1.max_relative_residual
    );

    // Why the scalar under-reads, in two factors, so the docs quote a measured
    // number instead of "the major axis dominates". `shell_scale` is the scalar's
    // denominator; the shape number's is the drawn half-width. Their ratio is the
    // *most* the scalar can miss by, before the direction of the residual is taken
    // into account — and it is not the aspect ratio, because the twelve shell
    // offsets are the state covariance's principal axes and their b-plane images do
    // not line up with the mapped ellipse's own.
    //
    // Printed at *both* ends of the knob on purpose: scaling the covariance scales
    // the shell and the ellipse together, so the factor should come out the same
    // number twice. That is what licenses quoting the shipping figure to explain a
    // gap measured at the top stop.
    for (label, (unc, report)) in ["shipping", "knob top"].iter().zip(checked.iter()) {
        let (major, minor) = unc.sigma_axes();
        let shell = report.shell_scale;
        println!(
            "{label} shell reaches {:.1} km = {:.3} of the {N_SIGMA}σ half-length ({:.1} km), and {:.0}x its half-width ({:.3} km)",
            shell / 1e3,
            shell / (N_SIGMA * major),
            N_SIGMA * major / 1e3,
            shell / (N_SIGMA * minor),
            N_SIGMA * minor / 1e3
        );
    }

    assert!(
        min_ship < SHIPPING_LIMIT,
        "the drawn minor axis at the shipping covariance is no longer supported by the \
         linearisation: residual is {min_ship:.4} of the {N_SIGMA}σ half-width \
         (limit {SHIPPING_LIMIT})"
    );
    assert!(
        min_top < KNOB_TOP_LIMIT,
        "at the σ knob's top stop the drawn minor axis is {min_top:.4} of its own \
         {N_SIGMA}σ half-width (limit {KNOB_TOP_LIMIT}) — past 1.0 the view draws a \
         width narrower than the error in it"
    );
    // The shape check has to be *stricter* than the shipping scalar, or it is not
    // adding anything. At the top stop the scalar reads ~0.004 while the axis it is
    // blind to reads ~0.56: two orders of magnitude apart, which is the whole
    // reason this test exists beside `bplane_uncertainty_checked`.
    //
    // Gated on the ellipse still being a needle, because the scalar's blindness is
    // *caused* by the aspect ratio — it normalises against the major axis. Make the
    // ellipse round (a bigger position block would) and the two numbers converge
    // legitimately, and an ungated assertion here would fail on a correct change
    // while blaming the normalisation.
    let (top_major, top_minor) = checked[1].0.sigma_axes();
    let aspect = top_major / top_minor;
    if aspect > 50.0 {
        assert!(
            min_top > checked[1].1.max_relative_residual * 10.0,
            "on a {aspect:.0}:1 ellipse the per-axis residual ({min_top:.4}) is no longer \
             telling us anything the shipping scalar ({:.4}) does not — check the \
             normalisation before trusting either",
            checked[1].1.max_relative_residual
        );
    }
    // And the major axis is where the scalar already looks, so the two should agree
    // in order of magnitude there.
    assert!(
        maj_ship < 0.01 && maj_top < 0.01,
        "the major axis residual grew unexpectedly (shipping {maj_ship:.4}, top {maj_top:.4})"
    );
}
