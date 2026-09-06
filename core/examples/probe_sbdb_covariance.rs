//! Measure the SBDB element→state covariance conversion, end to end, on the
//! committed Apophis record.
//!
//! `sbdb.rs`'s own tests assert that each gate *passes*. This prints the numbers
//! behind them, which is a different job: it is where the finite-difference step
//! plateau was measured, where the residual against JPL's own state can be read
//! rather than merely bounded, and where the resulting covariance can be put
//! beside the invented one the shipping campaign uses and compared.
//!
//! What it prints, and what would falsify each:
//!
//!   1. **The reconstruction residual** against JPL's Cartesian state at the
//!      covariance epoch, in both frames. A degrees-for-radians slip is ~1e10 m
//!      and a wrong `tp` scale ~1e9 m; what is actually left is metres.
//!   2. **The finite-difference plateau**, per column, over six decades of step.
//!      [`FD_RELATIVE_STEP`] must sit inside a flat region and not on its edge —
//!      the same standard `uncertainty`'s two steps were held to, except that
//!      this map is closed form, so the plateau is much wider.
//!   3. **The mapped covariance**: position and velocity σ, the cigar's aspect
//!      ratio and how far its long axis lies from the velocity direction, and
//!      the asymmetry the symmetrising pass absorbed.
//!   4. **The same object's covariance beside `synthetic_along_track`'s** at the
//!      shipping numbers — the invented shape against a measured one.
//!
//! Needs no kernels and no network.
//!
//!   cargo run -p asteroid_core --release --example probe_sbdb_covariance

use asteroid_core::{state_from_elements_si, SbdbOrbit, StateCovariance, FD_RELATIVE_STEP};
use nalgebra::Matrix6;

/// DE440's `μ_sun`, SI. A probe is allowed to name it: everything physical in
/// the crate reads `μ` from the loaded kernel, but this file deliberately runs
/// without one, and the residual it prints is the evidence that the choice costs
/// tens of metres rather than kilometres.
const MU_SUN: f64 = 1.327_124_400_18e20;

const LABELS: [&str; 6] = ["e", "q", "tp", "node", "peri", "i"];

fn main() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/apophis.sbdb");
    let orbit = match SbdbOrbit::read(path) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("could not read {path}: {e}");
            eprintln!("(regenerate with `python pyref/fetch_sbdb_covariance.py`)");
            std::process::exit(1);
        }
    };

    println!("== {} ==", orbit.name());
    println!(
        "  JPL orbit {} ({})",
        orbit.orbit_id(),
        orbit.solution_date()
    );
    println!("  arc: {}", orbit.observation_arc());
    println!("  marginalized over: {:?}", orbit.marginalized());
    println!(
        "  covariance epoch {:.1} s past J2000; orbit epoch {:.1} s past J2000 ({:.2} yr apart)",
        orbit.covariance_epoch().tdb_seconds_past_j2000(),
        orbit.orbit_epoch().tdb_seconds_past_j2000(),
        (orbit.orbit_epoch().tdb_seconds_past_j2000()
            - orbit.covariance_epoch().tdb_seconds_past_j2000())
            / (365.25 * 86_400.0)
    );

    println!("\n-- 1. reconstruction against JPL's own state --");
    let ours_ecl = orbit.state_ecliptic(MU_SUN).expect("converts");
    let ours_icrf = orbit.state_icrf(MU_SUN).expect("converts");
    for (frame, ours, truth) in [
        ("ecliptic", ours_ecl, orbit.truth_state_ecliptic()),
        ("ICRF    ", ours_icrf, orbit.truth_state_icrf()),
    ] {
        let dr = (ours.position - truth.position).norm();
        let dv = (ours.velocity - truth.velocity).norm();
        println!(
            "  {frame}: |Δr| = {dr:8.2} m ({:.2e} relative), |Δv| = {dv:.3e} m/s",
            dr / truth.position.norm()
        );
    }

    println!("\n-- 2. finite-difference plateau (relative step vs column) --");
    let base = orbit.fd_steps(MU_SUN).expect("steps");
    let reference = jacobian_at(&orbit, FD_RELATIVE_STEP);
    print!("  {:>10}", "rel step");
    for l in LABELS {
        print!(" {l:>12}");
    }
    println!("      <- max |ΔJ|/|J| against the shipping step");
    for decade in 2..=9 {
        let rel = 10f64.powi(-decade);
        let j = jacobian_at(&orbit, rel);
        print!("  {rel:>10.0e}");
        for k in 0..6 {
            let mut worst = 0.0_f64;
            for r in 0..6 {
                let denom = reference[(r, k)].abs().max(1e-30);
                worst = worst.max((j[(r, k)] - reference[(r, k)]).abs() / denom);
            }
            print!(" {worst:>12.2e}");
        }
        println!();
    }
    print!("  {:>10}", "abs step");
    for h in base {
        print!(" {h:>12.3e}");
    }
    println!("   (m, s, rad as appropriate)");
    println!(
        "  shipping FD_RELATIVE_STEP = {FD_RELATIVE_STEP:.0e}; it must sit in a flat \
         band, not on an edge"
    );

    println!("\n-- 3. the mapped covariance --");
    let mapped = orbit.state_covariance_icrf(MU_SUN).expect("maps");
    describe("SBDB (real)", mapped.covariance.matrix(), &orbit);
    println!(
        "  symmetrising absorbed {:.3e} ({:.2e} relative to the largest entry)",
        mapped.absorbed_asymmetry, mapped.absorbed_asymmetry_relative
    );

    println!("\n-- 3b. one element at a time, against the full covariance --");
    println!("    (if these are each far larger than the 6-element answer, the");
    println!("     element errors are correlated and cancel — which is the whole");
    println!("     reason a marginal covariance is not a list of sigmas)");
    let j = orbit.jacobian_ecliptic(MU_SUN).expect("jacobian");
    let sigma = orbit.covariance_si();
    let v_hat = ours_ecl.velocity.normalize();
    for k in 0..6 {
        let mut only = Matrix6::zeros();
        only[(k, k)] = sigma[(k, k)];
        let ecl = j * only * j.transpose();
        let pos = ecl.fixed_view::<3, 3>(0, 0).into_owned();
        let e = pos.symmetric_eigen();
        let mut hi = 0;
        for i in 1..3 {
            if e.eigenvalues[i] > e.eigenvalues[hi] {
                hi = i;
            }
        }
        let axis = e.eigenvectors.column(hi).normalize();
        let ang = axis.dot(&v_hat).abs().min(1.0).acos().to_degrees();
        println!(
            "    {:>5} alone: 1σ {:>10.1} m along an axis {:>6.2}° off velocity",
            LABELS[k],
            e.eigenvalues[hi].sqrt(),
            ang
        );
    }

    println!("\n-- 4. beside the invented one --");
    let truth = orbit.truth_state_icrf();
    match StateCovariance::synthetic_along_track(truth, 5.0e-5, 20.0, 1000.0) {
        Some(s) => describe("synthetic_along_track", s.matrix(), &orbit),
        None => println!("  synthetic_along_track refused this state"),
    }
    println!(
        "\n  The shipping campaign keeps the invented one — its rock has no \
         observation arc and never will.\n  What this record retires is the \
         invented label for an object that does."
    );
}

/// The Jacobian rebuilt at an arbitrary relative step, for the plateau sweep.
/// Duplicates `SbdbOrbit::jacobian_ecliptic`'s scheme on purpose: the shipping
/// method takes no step argument, and giving it one so a probe could sweep it
/// would put a knob on a constant that is only valid where it was measured.
fn jacobian_at(orbit: &SbdbOrbit, relative: f64) -> Matrix6<f64> {
    let x = orbit.elements_si();
    let base = orbit.fd_steps(MU_SUN).expect("steps");
    let mut j = Matrix6::zeros();
    for k in 0..6 {
        let h = base[k] / FD_RELATIVE_STEP * relative;
        let (mut p, mut m) = (x, x);
        p[k] += h;
        m[k] -= h;
        let sp = state_from(orbit, p);
        let sm = state_from(orbit, m);
        for r in 0..3 {
            j[(r, k)] = (sp[r] - sm[r]) / (2.0 * h);
            j[(3 + r, k)] = (sp[3 + r] - sm[3 + r]) / (2.0 * h);
        }
    }
    j
}

/// A perturbed element vector as a flat 6-state. Goes through the public API, so
/// this probe measures the shipping conversion and not a copy of it.
fn state_from(orbit: &SbdbOrbit, x: [f64; 6]) -> [f64; 6] {
    let _ = orbit;
    let s = state_from_elements_si(x, MU_SUN).expect("converts");
    [
        s.position.x,
        s.position.y,
        s.position.z,
        s.velocity.x,
        s.velocity.y,
        s.velocity.z,
    ]
}

/// Print a 6×6 state covariance as the things a reader can judge: the position
/// and velocity σ, the cigar, and where the cigar points.
fn describe(label: &str, m: &Matrix6<f64>, orbit: &SbdbOrbit) {
    let pos = m.fixed_view::<3, 3>(0, 0).into_owned();
    let vel = m.fixed_view::<3, 3>(3, 3).into_owned();
    let pe = pos.symmetric_eigen();
    let ve = vel.symmetric_eigen();
    let (mut hi, mut lo) = (0, 0);
    for i in 1..3 {
        if pe.eigenvalues[i] > pe.eigenvalues[hi] {
            hi = i;
        }
        if pe.eigenvalues[i] < pe.eigenvalues[lo] {
            lo = i;
        }
    }
    let long = pe.eigenvalues[hi].sqrt();
    let short = pe.eigenvalues[lo].sqrt();
    let axis = pe.eigenvectors.column(hi).normalize();
    let v_hat = orbit.truth_state_icrf().velocity.normalize();
    let angle = axis.dot(&v_hat).abs().min(1.0).acos().to_degrees();
    let v_long = ve.eigenvalues.max().sqrt();
    let v_short = ve.eigenvalues.min().sqrt();
    println!("  {label}:");
    // An isotropic position block has no long axis — its eigenvectors are
    // arbitrary, so printing an angle for one would be printing noise with two
    // decimal places on it. `synthetic_along_track` builds exactly that.
    let aspect = long / short;
    if aspect < 1.01 {
        println!(
            "    position 1σ: {long:.1} m, isotropic (aspect {aspect:.2}:1) — no \
             long axis, so no direction to report"
        );
    } else {
        println!(
            "    position 1σ: {long:.1} m long × {short:.1} m short (aspect \
             {aspect:.0}:1), long axis {angle:.2}° off velocity"
        );
    }
    println!("    velocity 1σ: {v_long:.3e} .. {v_short:.3e} m/s");
}
