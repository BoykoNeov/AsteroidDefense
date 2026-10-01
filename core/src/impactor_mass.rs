//! `impactor_mass` — how much of what a rocket delivers actually hits the rock
//! (HANDOFF §8, Phase 3's payload mass budget).
//!
//! [`crate::launch_vehicle`] answers *how much spacecraft can this rocket send to
//! this `C3`*. A kinetic impactor's push is `β·(m/M)·v_rel`, and the `m` in it is
//! the mass **at impact**, which is not the same number. This module is the one
//! place the two are told apart.
//!
//! # What comes off, and what does not
//! A kinetic impactor is unusual among spacecraft: its bus, structure, avionics,
//! power system and any propellant still in its tanks all hit the rock and all
//! carry momentum. So the budget is not "bus + propellant out of delivered mass",
//! which is what this layer's earlier docs feared. Only two things can leave:
//!
//! 1. **The launch adapter and separation system** — it stays on the upper stage.
//!    **Already out of the tables: nothing to subtract.** NASA's Launch Services
//!    Program states that its performance figures (the source of the AMAT tables
//!    [`crate::launch_vehicle`] embeds) are *separated spacecraft mass*, with the
//!    separation system book-kept on the launch-vehicle side:
//!    - SMEX 12/13/14 AO ELV Launch Services Program Information Summary
//!      (2007-11-19): "All of these figures reflect separated spacecraft mass and
//!      each have associated ground rules/assumptions (including the
//!      adapter-type)" and "Mass of entire separation system is book-kept on the
//!      launch vehicle side. Listed performance is for separated spacecraft mass."
//!      `explorers.larc.nasa.gov/PDF_FILES/AO-SMEX-ELVInfoSummary11_19_07.pdf`
//!    - MIDEX 2016 AO ELV summary, Rev A (2017-09-25): "Mass of entire separation
//!      system is book-kept on the launch vehicle side."
//!    - EVM-3 ELV LSP Information Summary (2020-09-30): "A representative
//!      separation system is assumed, the mass of which is book-kept on the launch
//!      vehicle side."
//! 2. **Propellant burned before impact** — trajectory corrections, attitude
//!    control, terminal targeting. The Lambert arcs this layer flies are ballistic
//!    (no deep-space burn), so this is the only real loss, and it is small.
//!
//! # The number: what DART actually lost, flown and measured
//! The one kinetic impactor that has flown gives the fraction directly:
//! - Launch mass **615 kg** — *Final Technical Report to NASA for the DART
//!   Mission* (APL, October 2023), Fig. 2 caption.
//! - Minus LICIACube, **≈14 kg**, the Italian cubesat DART carried and released
//!   on 11 September 2022, 15 days before impact (Final Technical Report §1) —
//!   mass from Dotto et al., *Planetary and Space Science* 199 (2021) 105185. It
//!   left the stack, but it is a passenger, not a loss a kinetic impactor would have.
//! - Mass at impact **579.4 ± 0.7 kg** — Cheng et al., *Nature* (2023),
//!   doi:10.1038/s41586-023-05878-z, Extended Data Table 1 (arXiv:2303.03464).
//!
//! So [`IMPACT_MASS_FRACTION`] `= 579.4 / (615 − 14) = 0.964`: 3.6 % of the
//! separated mass was burned on the way. That is everything DART spent — six TCMs,
//! terminal divert, attitude control and its NEXT-C ion-engine demonstration — so it
//! errs on the side of a *larger* deduction than a pure impactor needs.
//!
//! # The cross-check, from a second, independent source
//! The same report states DART's hydrazine was sized for "a total dV99 of 55.2
//! m/s" (§2.1.1), on MR-103G thrusters rated 202–224 s specific impulse
//! (manufacturer data as listed on SatCatalog, `satcatalog.com/component/mr-103g-1n/`).
//! Through the rocket equation that budget burns 2.5–2.8 % of the spacecraft — the
//! same few percent, and *below* the flown 3.6 % because a Δv budget leaves out
//! attitude control and the xenon. The test
//! `the_flown_fraction_agrees_with_the_delta_v_budget` pins that ordering, so an
//! edit to either side that broke their agreement would fail.
//!
//! # Scale-free, and why that is the right assumption here
//! Propellant for a fixed Δv is a fixed *fraction* of the spacecraft
//! (`1 − e^(−Δv/(Isp·g0))`), so the 3.6 % carries from DART's 0.6 t to a 14 t
//! Falcon Heavy impactor without a scaling law. What would break it is a mission
//! that needs a *different Δv* — a deep-space manoeuvre, or a rendezvous — and
//! this layer flies neither.
//!
//! # What this does not model
//! A design that separates a large part of itself before impact (an observer
//! spacecraft riding along, as in ESA's Don Quijote concept) would lose more. This
//! layer assumes a single impactor, which is what DART was once LICIACube is taken
//! out.

/// DART's mass at launch, kg — APL, *DART Final Technical Report* (Oct 2023), Fig. 2.
pub const DART_LAUNCH_MASS_KG: f64 = 615.0;

/// LICIACube's mass, kg — "approximately 14 kg", Dotto et al., PSS 199 (2021) 105185.
/// Released before impact; a passenger, not part of the impactor.
pub const LICIACUBE_MASS_KG: f64 = 14.0;

/// DART's mass at impact, kg — Cheng et al., Nature (2023), Extended Data Table 1 (±0.7).
pub const DART_IMPACT_MASS_KG: f64 = 579.4;

/// The fraction of a launcher's **separated** spacecraft mass that is still there
/// at impact: DART's flown ratio with its passenger cubesat removed, `0.964`.
/// Derived from the three cited masses, never written down on its own.
pub const IMPACT_MASS_FRACTION: f64 =
    DART_IMPACT_MASS_KG / (DART_LAUNCH_MASS_KG - LICIACUBE_MASS_KG);

/// DART's 99th-percentile hydrazine Δv budget, m/s — *DART Final Technical Report*,
/// §2.1.1. Used only to cross-check [`IMPACT_MASS_FRACTION`].
pub const DART_HYDRAZINE_DV99_M_S: f64 = 55.2;

/// MR-103G (DART's hydrazine thrusters) steady-state specific impulse range, s —
/// SatCatalog's listing of the manufacturer's data. Cross-check only.
pub const MR103G_ISP_RANGE_S: (f64, f64) = (202.0, 224.0);

/// Standard gravity, m/s² (the definition the specific impulse is quoted against).
const G0_M_S2: f64 = 9.806_65;

/// The mass that hits the rock when a launcher delivers `separated_kg` of
/// spacecraft, kg. `0` stays `0`: a launcher that cannot reach a `C3` delivers no
/// impactor.
pub fn impact_mass_kg(separated_kg: f64) -> f64 {
    separated_kg * IMPACT_MASS_FRACTION
}

/// The fraction of a spacecraft a Δv of `dv_m_s` burns at specific impulse
/// `isp_s` — the rocket equation, `1 − e^(−Δv/(Isp·g0))`.
pub fn propellant_fraction(dv_m_s: f64, isp_s: f64) -> f64 {
    1.0 - (-dv_m_s / (isp_s * G0_M_S2)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped fraction is the cited arithmetic, to the digit — so editing a
    /// source mass without the fraction (or the reverse) cannot go unnoticed.
    #[test]
    fn the_fraction_is_dart_with_its_passenger_removed() {
        let f = 579.4 / (615.0 - 14.0);
        assert_eq!(IMPACT_MASS_FRACTION, f);
        assert!(
            (IMPACT_MASS_FRACTION - 0.9641).abs() < 1e-4,
            "{IMPACT_MASS_FRACTION}"
        );
        // Reproduces DART's own impact mass from its separated mass.
        let separated = DART_LAUNCH_MASS_KG - LICIACUBE_MASS_KG;
        assert!((impact_mass_kg(separated) - DART_IMPACT_MASS_KG).abs() < 1e-9);
    }

    /// The flown loss and the Δv budget are two independent sources for the same
    /// few percent. The budget leaves out attitude control and the ion-engine demo,
    /// so it must sit *below* the flown loss — but in the same range, not an order
    /// of magnitude off. Either failing means a source was misread.
    #[test]
    fn the_flown_fraction_agrees_with_the_delta_v_budget() {
        let flown_loss = 1.0 - IMPACT_MASS_FRACTION;
        let (isp_lo, isp_hi) = MR103G_ISP_RANGE_S;
        let budget_hi = propellant_fraction(DART_HYDRAZINE_DV99_M_S, isp_lo);
        let budget_lo = propellant_fraction(DART_HYDRAZINE_DV99_M_S, isp_hi);
        assert!((0.024..0.026).contains(&budget_lo), "{budget_lo}");
        assert!((0.027..0.029).contains(&budget_hi), "{budget_hi}");
        assert!(
            budget_hi < flown_loss,
            "budget {budget_hi} vs flown {flown_loss}"
        );
        assert!(
            flown_loss < 2.0 * budget_lo,
            "flown {flown_loss} vs budget {budget_lo}"
        );
    }

    #[test]
    fn nothing_delivered_is_nothing_at_impact() {
        assert_eq!(impact_mass_kg(0.0), 0.0);
        assert!((impact_mass_kg(10_000.0) - 10_000.0 * IMPACT_MASS_FRACTION).abs() < 1e-9);
    }
}
