//! `orbital_assembly` — launching to a parking orbit and leaving later (HANDOFF §8,
//! Phase 3's orbital assembly).
//!
//! The campaign layer ([`crate::campaign`]) sends every launch straight to the rock:
//! the rocket's own upper stage does the escape burn, so a launch leaves on its
//! launch date and carries what the NASA tables say it can to that window's `C3`.
//! This module supplies the other way to fly a launch — up to a parking orbit first,
//! out on a later date with a departure stage of its own — and the one number that
//! way is priced by: **the mass that leaves Earth, per launch, as a function of the
//! departure's `C3`.**
//!
//! # Joining the pieces buys nothing; waiting for a date is what can pay
//! A kinetic impactor's push is `β·(m/M)·v_rel`, linear in mass, and at a real
//! launch's millimetres per second the b-plane responds linearly too (the campaign
//! layer measures that rather than assuming it). So one stack of `N` launches' mass
//! through a window pushes exactly as hard as `N` separate impactors through the
//! same window — and the campaign planner already allows `N` launches on one date.
//! *Joining* modules in orbit is therefore not where a gain can come from.
//!
//! What a parking orbit changes is **when a launch can leave**. Straight to the rock,
//! a launch leaves on its launch date, so a launch cap of one a year forces late
//! launches through late, weak windows. Parked, a launch can wait for the best
//! departure after it. That is worth something only where the cap pushes launches
//! onto weak dates — measured on the shipping rock, at one a year and nowhere else.
//!
//! # And waiting costs mass
//! A rocket's upper stage cannot wait months in orbit, so a parked payload must
//! carry its own departure stage, and the escape burn comes out of the payload. The
//! per-launch mass is the rocket equation on the parked stack:
//!
//! `m_out = m_stack · e^(−Δv / (Isp · g0))`, with `Δv = √(2μ/r + C3) − √(μ/r)`
//!
//! the impulsive burn from a circular parking orbit of radius `r` to a hyperbola of
//! characteristic energy `C3`. **The spent stage stays attached and hits the rock**
//! — the same reasoning as [`crate::impactor_mass`]: everything that arrives carries
//! momentum, and only burned propellant leaves. So no stage dry mass is subtracted;
//! the stage's tankage is impactor mass, and `m_out` is the *separated* mass in
//! the sense the launch tables use, to which [`crate::impactor_mass::impact_mass_kg`]
//! applies as it does to a direct launch.
//!
//! # The stack is capped by the payload adapter, not by the rocket
//! SpaceX advertises 63.8 t to low Earth orbit for Falcon Heavy expendable. Its own
//! user's guide caps a single payload far lower: the largest published interface,
//! the 3,117-mm strut payload attach fitting (extended fairing only), is listed at
//! **"Total Mass: up to 26,500 kg"** — *Falcon User's Guide* (SpaceX, 2025-05-09),
//! Table 4-1, *Guide on PAF Selection and Payload and Adapter Mass Overall
//! Limitations*, which calls its masses "an initial guide" ("Further limitations may
//! exist from mission-specific analyses"). The same guide says mass-to-orbit
//! figures are "available upon request" — it publishes no low-orbit capability. So
//! the shipping stack is [`FALCON_SINGLE_PAYLOAD_LIMIT_KG`]: a Falcon Heavy can
//! certainly lift it (the NASA tables have it sending 14.7 t all the way to
//! `C3 = 1`), and nothing published lets one payload be heavier. The 63.8 t figure
//! is kept as [`FALCON_HEAVY_ADVERTISED_LEO_KG`] for a labelled *what if* only.
//!
//! # The departure engine: storable, because the stack waits up to a year or more
//! On the shipping rock the best parked plans wait up to ~1.3 years in orbit before
//! leaving. Liquid hydrogen boils off over that time, and this project has no
//! sourced boil-off rate to say how much would be left. So the shipping stage burns
//! storable propellant (MMH and MON-3, which keep): [`ORION_OMS_E`], the Space Shuttle
//! orbital manoeuvring engine refurbished for Orion's European Service Module, flown
//! on Artemis I — nominal specific impulse **315.1 s**, Belair et al., *Artemis I
//! Orion-ESM Propulsion System Engine Performance*, Space Propulsion 2024
//! (SP2024_382; NASA Glenn, ESA, Airbus), Table 2 (`ntrs.nasa.gov`, citation
//! 20240003648). A hydrogen engine ([`RL10B_2`], 465.5 s, National Research Council,
//! *A Review of United States Air Force and Department of Defense Aerospace
//! Propulsion Needs* (2006), Appendix D, p. 256) is carried as a *what if* — the
//! answer if boil-off were solved, not a claim that it is.
//!
//! # The parking orbit
//! 185 km circular — the perigee altitude the Falcon guide baselines for its
//! transfer orbits (§3.1, "A perigee altitude of 185 km (100 nmi) is baselined for
//! GTO"). It is the **conservative** choice for the escape burn: from higher up the
//! burn is smaller (about 45 m/s less from 400 km at `C3 = 10`), and a stack waiting
//! a year would in practice sit higher, because a 185 km orbit decays in weeks.
//!
//! # What this does not model - and two of it favour parking
//! - **The parking orbit's plane.** The departure asymptote has to lie in (or be
//!   reached from) the parking orbit's plane on the departure date. A direct launch
//!   gets that by choosing its launch time on the day; a parked stack's plane is
//!   fixed at launch and regresses with Earth's oblateness (a ~50-day cycle at
//!   185 km), so it can need a plane change or a date off the window's best. Not
//!   charged here - a bias toward parking.
//! - **Gravity losses.** The burn is impulsive. A 26.7 kN storable engine pushing a
//!   26.5 t stack burns for ~40 minutes at `C3` ~43 - past the OMS-E's rated
//!   1 030 s maximum, so at least three perigee burns - and a burn that long costs
//!   real Δv, plausibly tens of m/s. Also a bias toward parking.
//! - **Orbit decay at 185 km** (a real stack would wait higher, which needs *less*
//!   burn) and **rendezvous and docking** (two parked launches leaving the same date
//!   are counted as joined at no mass cost; by linearity they could equally leave as
//!   two stacks on that date).

use crate::impactor_mass::{impact_mass_kg, propellant_fraction};
use crate::launch_vehicle::{LaunchVehicle, FALCON_HEAVY_EXPENDABLE};

/// The largest published single-payload mass for a Falcon launch, kg — the 3,117-mm
/// strut PAF, extended fairing only: "Total Mass: up to 26,500 kg". *Falcon User's
/// Guide* (SpaceX, 2025-05-09), Table 4-1 ("Masses should be used as an initial
/// guide").
pub const FALCON_SINGLE_PAYLOAD_LIMIT_KG: f64 = 26_500.0;

/// Falcon Heavy expendable's **advertised** payload to low Earth orbit, kg, as
/// SpaceX's vehicle page states it. Not a published single-payload limit (see
/// [`FALCON_SINGLE_PAYLOAD_LIMIT_KG`]) and never flown — a labelled *what if* only.
pub const FALCON_HEAVY_ADVERTISED_LEO_KG: f64 = 63_800.0;

/// Parking-orbit altitude, m — the Falcon guide's baselined transfer-orbit perigee
/// (§3.1). The conservative choice for the escape burn (see the module doc).
pub const PARKING_ALTITUDE_M: f64 = 185_000.0;

/// Earth's GM, m³/s² — the DE440-consistent literal the kernel-free geometry tests
/// use. The escape burn moves by well under a metre per second across every
/// published value.
pub const EARTH_GM_M3_S2: f64 = 3.986_004_356e14;

/// A departure stage's engine, as far as the escape burn needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepartureEngine {
    /// Display name.
    pub name: &'static str,
    /// Vacuum specific impulse, s.
    pub isp_s: f64,
    /// Whether its propellants keep in orbit for the year or more a parked stack
    /// waits. Only a storable engine is a shipping default; see the module doc.
    pub storable: bool,
}

/// Orion's main engine (the Shuttle OMS engine, MMH / MON-3): nominal 315.1 s —
/// Belair et al., Space Propulsion 2024, SP2024_382, Table 2.
pub const ORION_OMS_E: DepartureEngine = DepartureEngine {
    name: "OMS-E (storable)",
    isp_s: 315.1,
    storable: true,
};

/// RL10B-2 (liquid hydrogen / liquid oxygen): 465.5 s — National Research Council
/// (2006), Appendix D, p. 256. Hydrogen boils off; a *what if* only.
pub const RL10B_2: DepartureEngine = DepartureEngine {
    name: "RL10B-2 (hydrogen)",
    isp_s: 465.5,
    storable: false,
};

/// One way to fly a launch through a parking orbit: how much stack one launch puts
/// up, the engine that takes it out, and the orbit it waits in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParkedDelivery {
    /// Display name.
    pub name: &'static str,
    /// Mass one launch puts in the parking orbit, kg — stage, propellant and
    /// impactor together.
    pub stack_kg: f64,
    /// The departure engine.
    pub engine: DepartureEngine,
    /// Circular parking-orbit altitude, m.
    pub parking_altitude_m: f64,
}

impl ParkedDelivery {
    /// The escape burn from the parking orbit to characteristic energy `c3_km2_s2`
    /// (km²/s², the launch tables' unit), m/s — impulsive, from circular speed to
    /// the hyperbola's perigee speed.
    pub fn departure_dv_m_s(&self, c3_km2_s2: f64) -> f64 {
        let r = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + self.parking_altitude_m;
        let c3 = c3_km2_s2 * 1.0e6;
        (2.0 * EARTH_GM_M3_S2 / r + c3).sqrt() - (EARTH_GM_M3_S2 / r).sqrt()
    }

    /// The mass that leaves the parking orbit toward the rock, kg — the stack less
    /// the propellant the escape burn takes. The spent stage stays on, so this is
    /// the *separated* mass in the launch tables' sense. `0` for a non-finite `C3`.
    pub fn separated_mass_kg(&self, c3_km2_s2: f64) -> f64 {
        if !c3_km2_s2.is_finite() {
            return 0.0;
        }
        let dv = self.departure_dv_m_s(c3_km2_s2);
        self.stack_kg * (1.0 - propellant_fraction(dv, self.engine.isp_s))
    }

    /// The mass that hits the rock, kg: [`separated_mass_kg`](Self::separated_mass_kg)
    /// through the same cruise loss a direct launch takes.
    pub fn impact_mass_kg(&self, c3_km2_s2: f64) -> f64 {
        impact_mass_kg(self.separated_mass_kg(c3_km2_s2))
    }
}

/// The shipping parked launch: the published single-payload limit, a storable
/// engine, the guide's 185 km orbit.
pub const SHIPPING_PARKED_DELIVERY: ParkedDelivery = ParkedDelivery {
    name: "26.5 t stack, storable",
    stack_kg: FALCON_SINGLE_PAYLOAD_LIMIT_KG,
    engine: ORION_OMS_E,
    parking_altitude_m: PARKING_ALTITUDE_M,
};

/// The parked delivery `vehicle` can fly, or `None` where nothing sourced says
/// what one payload of it may weigh in a parking orbit.
///
/// A parked stack needs two sourced halves: a **single-payload limit** (the most
/// one payload may weigh) and evidence the rocket **lifts that much to low orbit**.
/// Only Falcon Heavy expendable has both - the Falcon guide's 26 500 kg, and
/// SpaceX's advertised 63.8 t to low orbit, well above it. Falcon Heavy *reusable*
/// shares the guide's adapter limit but has no published low-orbit figure; Atlas V,
/// Vulcan and Delta IV Heavy have neither here. Giving any of them the Falcon stack
/// would invent a gain (a 26.5 t stack is far more than an Atlas V carries), so they
/// fly straight to the rock only.
///
/// Matched by name, not by address: a `const` has no stable address to compare.
pub fn parked_delivery_for(vehicle: &LaunchVehicle) -> Option<ParkedDelivery> {
    (vehicle.name == FALCON_HEAVY_EXPENDABLE.name).then_some(SHIPPING_PARKED_DELIVERY)
}

/// The labelled *what ifs*: a hydrogen engine that does not boil off, the
/// advertised low-orbit figure as one payload, and both. Reported next to the
/// shipping answer, never in place of it.
pub const WHAT_IF_PARKED_DELIVERIES: [ParkedDelivery; 3] = [
    ParkedDelivery {
        name: "26.5 t stack, hydrogen",
        stack_kg: FALCON_SINGLE_PAYLOAD_LIMIT_KG,
        engine: RL10B_2,
        parking_altitude_m: PARKING_ALTITUDE_M,
    },
    ParkedDelivery {
        name: "63.8 t stack, storable",
        stack_kg: FALCON_HEAVY_ADVERTISED_LEO_KG,
        engine: ORION_OMS_E,
        parking_altitude_m: PARKING_ALTITUDE_M,
    },
    ParkedDelivery {
        name: "63.8 t stack, hydrogen",
        stack_kg: FALCON_HEAVY_ADVERTISED_LEO_KG,
        engine: RL10B_2,
        parking_altitude_m: PARKING_ALTITUDE_M,
    },
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::launch_vehicle::FALCON_HEAVY_EXPENDABLE;

    /// The burn is the vis-viva difference: at `C3 = 0` it is exactly escape minus
    /// circular speed (~3.22 km/s from 185 km), and it grows with `C3`.
    #[test]
    fn the_escape_burn_is_vis_viva() {
        let d = SHIPPING_PARKED_DELIVERY;
        let r = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + PARKING_ALTITUDE_M;
        let v_c = (EARTH_GM_M3_S2 / r).sqrt();
        let dv0 = d.departure_dv_m_s(0.0);
        assert!((dv0 - (2.0_f64.sqrt() - 1.0) * v_c).abs() < 1e-9, "{dv0}");
        assert!((3_200.0..3_250.0).contains(&dv0), "{dv0}");
        let mut last = dv0;
        for c3 in [1.0, 10.0, 40.0, 80.0] {
            let dv = d.departure_dv_m_s(c3);
            assert!(dv > last, "C3 {c3}: {dv} not above {last}");
            last = dv;
        }
        // From higher up the burn is smaller: 185 km is the conservative choice.
        let high = ParkedDelivery {
            parking_altitude_m: 400_000.0,
            ..d
        };
        let saved = d.departure_dv_m_s(10.0) - high.departure_dv_m_s(10.0);
        assert!((30.0..60.0).contains(&saved), "{saved}");
    }

    /// The rocket equation, to the digit, and the stage stays on: nothing but
    /// propellant leaves.
    #[test]
    fn the_mass_out_is_the_rocket_equation_with_the_stage_kept() {
        let d = SHIPPING_PARKED_DELIVERY;
        for c3 in [0.0, 12.0, 38.0] {
            let dv = d.departure_dv_m_s(c3);
            let expect = d.stack_kg * (-dv / (d.engine.isp_s * 9.806_65)).exp();
            assert!((d.separated_mass_kg(c3) - expect).abs() < 1e-6);
            assert!(
                (d.impact_mass_kg(c3) - impact_mass_kg(expect)).abs() < 1e-6,
                "the cruise loss applies as it does to a direct launch"
            );
        }
        assert_eq!(d.separated_mass_kg(f64::NAN), 0.0);
    }

    /// Parking is offered only where both halves are sourced: Falcon Heavy
    /// expendable, and no other launcher on the table.
    #[test]
    fn only_falcon_heavy_expendable_parks() {
        use crate::launch_vehicle::LAUNCH_VEHICLES;
        let parks: Vec<&str> = LAUNCH_VEHICLES
            .iter()
            .filter(|v| parked_delivery_for(v).is_some())
            .map(|v| v.name)
            .collect();
        assert_eq!(parks, vec![FALCON_HEAVY_EXPENDABLE.name]);
        assert_eq!(
            parked_delivery_for(&FALCON_HEAVY_EXPENDABLE),
            Some(SHIPPING_PARKED_DELIVERY)
        );
    }

    /// The shipping stack is the published adapter limit with a storable engine, and
    /// every *what if* is labelled as one: each carries more than the shipping
    /// stack, and either a hydrogen engine or the advertised figure.
    #[test]
    fn the_shipping_choice_is_the_published_one() {
        let s = SHIPPING_PARKED_DELIVERY;
        assert_eq!(s.stack_kg, FALCON_SINGLE_PAYLOAD_LIMIT_KG);
        assert!(s.engine.storable);
        for w in WHAT_IF_PARKED_DELIVERIES {
            assert!(!w.engine.storable || w.stack_kg > FALCON_SINGLE_PAYLOAD_LIMIT_KG);
            assert!(
                w.separated_mass_kg(10.0) > s.separated_mass_kg(10.0),
                "{}",
                w.name
            );
        }
    }

    /// Parking **costs** mass wherever the shipping rock's windows sit: against a
    /// direct Falcon Heavy, the shipping parked launch carries less at every `C3`
    /// up to the crossing, and the crossing sits near the rocket's energy limit,
    /// above nearly every window the campaign flies — so a parked launch mostly wins
    /// by leaving on a better date, not by carrying more. Above the crossing it does
    /// carry more, because the direct curve falls steeply near that limit.
    #[test]
    fn parking_costs_mass_below_the_crossing() {
        let s = SHIPPING_PARKED_DELIVERY;
        let fh = FALCON_HEAVY_EXPENDABLE;
        let crossing = (0..=1_000)
            .map(|k| f64::from(k) * 0.1)
            .find(|&c3| s.separated_mass_kg(c3) >= fh.payload_kg(c3))
            .expect("the curves cross below C3 = 100");
        assert!(
            (56.0..70.0).contains(&crossing),
            "crossing at C3 {crossing}"
        );
        for c3 in [1.0, 10.0, 20.0, 38.4, 50.0] {
            assert!(s.separated_mass_kg(c3) < fh.payload_kg(c3), "C3 {c3}");
        }
    }
}
