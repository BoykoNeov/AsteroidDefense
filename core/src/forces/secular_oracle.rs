//! Test-only oracles for the **secular** (orbit-averaged) semi-major-axis drift a
//! transverse acceleration produces — shared by the Yarkovsky and gravity-tractor
//! suites (HANDOFF §6).
//!
//! # Why one oracle can judge two terms
//! [`super::yarkovsky::YarkovskyA2`] applies `A2·(r₀/r)^d · t̂` and
//! [`super::tractor::GravityTractor`] applies `G·m/d² · t̂`. Those look like
//! different physics — one is thermal recoil that fades with heliocentric
//! distance, the other is a spacecraft's gravity — but their *dynamical* content
//! is identical: a transverse in-plane acceleration whose only distinguishing
//! parameter is how it scales with `r`. The tractor's magnitude does not scale
//! with `r` at all, because station-keeping holds the spacecraft at a **fixed**
//! separation from the asteroid regardless of where the pair is in the solar
//! system. So the tractor is exactly the `d = 0` case of the Yarkovsky
//! parametrization, and [`time_averaged`] covers both by taking `d` as an
//! argument.
//!
//! That is worth stating plainly rather than hiding, because "the tractor is just
//! Yarkovsky with a window" sounds like a criticism and is actually the reason
//! the new term needs **no new oracle**: it inherits the strongest validation
//! machinery in the force suite. What is genuinely new in the tractor — the time
//! window, the `G·m/d²` coupling, and the station-keeping feasibility constraint
//! — is precisely what this module does *not* cover, and is tested separately.
//!
//! # Why this lives here instead of inside one term's test module
//! It began as a private helper in `yarkovsky.rs`'s `#[cfg(test)] mod tests`.
//! Copying it into the tractor's tests would have created two implementations of
//! the same Gauss planetary equation, free to drift apart under a later edit —
//! the failure mode this codebase has hit before (the `SB441_BODIES` drift test
//! exists for the same reason). One implementation, two callers.

/// The **time-averaged** secular `da/dt` for a transverse `a_T = A·(r₀/r)^d`
/// acceleration, computed straight from the Gauss planetary equation —
/// independently of any [`super::ForceModel`] implementation. The transverse
/// magnitude comes from the `A·(r₀/r)^d` scalar directly and never from a term's
/// `acceleration()`, so a magnitude or direction bug in the term under test
/// cannot cancel against the oracle.
///
/// ```text
/// da/dt = (2 / (n·√(1−e²))) · [ e·sinν·a_R + (p/r)·a_T ],    a_R = 0 here
/// ```
///
/// Sampling uniformly in **mean anomaly** `M` is uniform in *time* by
/// construction — the weighting that is make-or-break for this oracle: a
/// uniform-in-true-anomaly average would be wrong by ~10 % at `e ≈ 0.2`, which is
/// large enough to "validate" a broken term.
///
/// `d = 2` is the Yarkovsky/JPL `A2` convention; `d = 0` is a constant transverse
/// push (a gravity tractor station-keeping at fixed separation).
pub fn time_averaged(amplitude: f64, r0: f64, d: f64, a: f64, e: f64, mu: f64) -> f64 {
    let n = (mu / (a * a * a)).sqrt(); // mean motion
    let samples = 4000;
    let mut sum = 0.0;
    for i in 0..samples {
        let m = std::f64::consts::TAU * (i as f64) / (samples as f64);
        // Solve Kepler M = E − e·sinE for E (Newton; e is modest).
        let mut ecc = m;
        for _ in 0..60 {
            let f = ecc - e * ecc.sin() - m;
            let fp = 1.0 - e * ecc.cos();
            ecc -= f / fp;
        }
        let r = a * (1.0 - e * ecc.cos());
        // p/r = 1 + e·cosν, with p = a(1−e²).
        let one_plus_ecos_nu = a * (1.0 - e * e) / r;
        let a_t = amplitude * (r0 / r).powf(d);
        sum += 2.0 / (n * (1.0 - e * e).sqrt()) * one_plus_ecos_nu * a_t;
    }
    sum / (samples as f64)
}

/// Closed form of the same average for the **Yarkovsky** exponent `d = 2`:
/// `2·A2·r₀²/(n·a²·(1−e²))`. A cross-check on [`time_averaged`]'s weighting
/// algebra at the exponent the shipping Yarkovsky term uses.
pub fn closed_form_inverse_square(a2: f64, r0: f64, a: f64, e: f64, mu: f64) -> f64 {
    let n = (mu / (a * a * a)).sqrt();
    2.0 * a2 * r0 * r0 / (n * a * a * (1.0 - e * e))
}

/// Closed form of the same average for a **constant** transverse acceleration
/// (`d = 0`) on a **circular** orbit: `da/dt = 2·a_T/n`.
///
/// Deliberately restricted to `e = 0`, where `p/r ≡ 1` makes the time-average
/// exact with no weighting argument at all. This is the gravity tractor's
/// de-risk case: it pins the `d = 0` path of [`time_averaged`] against arithmetic
/// simple enough to check by eye, so the eccentric tractor case can then lean on
/// the numerical oracle without that oracle being the *only* witness at `d = 0`.
pub fn closed_form_constant_circular(a_t: f64, a: f64, mu: f64) -> f64 {
    let n = (mu / (a * a * a)).sqrt();
    2.0 * a_t / n
}

/// Osculating semi-major axis from a heliocentric state, via vis-viva:
/// `a = 1/(2/r − v²/μ)`. The quantity both secular suites sample
/// stroboscopically.
pub fn osculating_a(state: &crate::state::StateVector, mu: f64) -> f64 {
    let r = state.position.norm();
    let v2 = state.velocity.norm_squared();
    1.0 / (2.0 / r - v2 / mu)
}

/// Least-squares slope of `ys` against integer index `0..n` — the drift-per-step
/// estimator both suites apply to a stroboscopic series.
pub fn slope_per_step(ys: &[f64]) -> f64 {
    let n = ys.len() as f64;
    let mean_x = (n - 1.0) / 2.0;
    let mean_y = ys.iter().sum::<f64>() / n;
    let (mut num, mut den) = (0.0, 0.0);
    for (i, &y) in ys.iter().enumerate() {
        let dx = i as f64 - mean_x;
        num += dx * (y - mean_y);
        den += dx * dx;
    }
    num / den
}

#[cfg(test)]
mod tests {
    use super::*;

    const MU_SUN: f64 = 1.327_124_400_18e20;
    const AU: f64 = 1.495_978_707e11;

    /// The two independent oracles must agree at the Yarkovsky exponent —
    /// validates the uniform-`M` weighting before either is trusted to judge a
    /// term. (Moved here with the oracle it guards; it was `yarkovsky.rs`'s
    /// `oracle_time_average_matches_the_closed_form`.)
    #[test]
    fn time_average_matches_the_inverse_square_closed_form() {
        for &e in &[0.0, 0.2, 0.45] {
            let a = 1.1 * AU;
            let num = time_averaged(1e-9, AU, 2.0, a, e, MU_SUN);
            let cf = closed_form_inverse_square(1e-9, AU, a, e, MU_SUN);
            let rel = (num - cf).abs() / cf.abs();
            assert!(
                rel < 1e-4,
                "e={e}: numerical {num} vs closed form {cf} (rel {rel:.2e})"
            );
        }
    }

    /// The same agreement at the **tractor's** exponent `d = 0`, on the circular
    /// case where the closed form is exact. Without this, `d = 0` would be a code
    /// path the oracle exercises for the first time while simultaneously being
    /// the only witness for the term it judges.
    #[test]
    fn time_average_matches_the_constant_closed_form_when_circular() {
        let a_t = 3e-11;
        let a = 0.9 * AU;
        let num = time_averaged(a_t, AU, 0.0, a, 0.0, MU_SUN);
        let cf = closed_form_constant_circular(a_t, a, MU_SUN);
        let rel = (num - cf).abs() / cf.abs();
        assert!(
            rel < 1e-6,
            "numerical {num} vs closed form {cf} (rel {rel:.2e})"
        );
    }

    /// `d = 0` must actually *be* distance-independent: the same amplitude on a
    /// markedly eccentric orbit gives a drift that does not depend on `r₀`, while
    /// the `d = 2` oracle plainly does. Guards against a caller silently getting
    /// the Yarkovsky scaling when it asked for a constant tug.
    #[test]
    fn constant_exponent_ignores_the_reference_distance() {
        let (a, e) = (1.2 * AU, 0.3);
        let at_au = time_averaged(3e-11, AU, 0.0, a, e, MU_SUN);
        let at_half = time_averaged(3e-11, 0.5 * AU, 0.0, a, e, MU_SUN);
        assert_eq!(
            at_au, at_half,
            "d=0 must be independent of r₀, got {at_au} vs {at_half}"
        );
        // The d=2 control: same change of r₀ must move the answer a lot (×4).
        let sq_au = time_averaged(3e-11, AU, 2.0, a, e, MU_SUN);
        let sq_half = time_averaged(3e-11, 0.5 * AU, 2.0, a, e, MU_SUN);
        assert!(
            (sq_au / sq_half - 4.0).abs() < 1e-9,
            "d=2 must scale as r₀², got ratio {}",
            sq_au / sq_half
        );
    }
}
