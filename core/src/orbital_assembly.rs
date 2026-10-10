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
//! characteristic energy `C3` - plus, for the shipping engine, the **loss** a
//! finite burn split into firings pays on top (below). **The spent stage stays attached and hits the rock**
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
//! # The parking orbit: 400 km, because the stack has to survive the wait
//! The shipping parked plans wait up to ~16 months. The Falcon guide's baselined
//! transfer-orbit perigee, 185 km (§3.1, "A perigee altitude of 185 km (100 nmi)
//! is baselined for GTO"), decays in weeks, so a stack cannot wait there; the
//! shipping orbit is **400 km**, about the Space Station's height, where a stack
//! can be held for that long. It is a stated choice, not a sourced figure. It
//! costs the launcher nothing that matters here: the stack is capped by the payload
//! adapter at 26.5 t, far under the 63.8 t Falcon Heavy is advertised to lift to
//! low orbit, so the extra ~120 m/s of ascent fits easily. And it *helps* the
//! escape burn, which is smaller from higher up. 185 km was the first cut and is
//! kept in the bracket only.
//!
//! **The 1-a-year answer is on the edge, and the height decides it.** 400 km is
//! the *lowest* height a stack survives at, not a natural one, and every height
//! above it needs less escape burn. Measured at the shipping escape setting: 1 in
//! any 12 months falls short at 400 km (by 38 km of the 1 % band), sits on the line
//! at 450 km (short by 4 km on windows chosen for 400 km, at it by 4 km on its own -
//! within the ranking noise, 2026-10-08), is at the line from 500 km, and clears
//! the target outright from about
//! 1 000 km (which leans on an unsourced lift that high, at the edge of the
//! radiation belts). The user kept 400 km as the cautious default (2026-10-07), so
//! the shipping answer was "short" and quoted with this edge.
//!
//! **Superseded 2026-10-08:** all of the above was measured with launches stopping
//! 3.6 yr before impact, the launch map's old axis end - a drawing choice that the
//! 1-a-year plan was pressed against. With the axes stretched (HANDOFF *Standing
//! defence: when the first rocket can fly*), 1 in any 12 months at 400 km is **10
//! launches, clear by 1.2 %**. Re-run there (2026-10-08, the user's call): **10
//! launches from 400 to 800 km, 9 from 1 000 km up** (|B| 26 052 at 1 000 km), 2..12
//! all 6 at every height. So the height now decides 10 against 9, not short against
//! clear, and 400 km stays the shipping choice.
//!
//! # The escape burn is flown finite ([`crate::departure_burn`])
//! The OMS-E pushes 26.7 kN against a 26.5 t stack, rated for at most 1 030 s per
//! firing and 10 starts, so the escape is split into firings at perigee and the
//! time spent off the ideal point costs propellant - the **loss**. Shipping:
//! [`SHIPPING_ESCAPE_FIRINGS`] = 8 firings (two of the 10 starts kept for trims
//! during the wait) and no intermediate apogee past [`SHIPPING_APOGEE_CAP_M`] =
//! 100 000 km, where the Moon's tidal pull is 0.04 % of Earth's and a two-body
//! flight can be trusted. About 35 m/s at `C3` 43 from 400 km. Chosen by the user
//! (2026-10-07) from the bracket the probe prints: more firings and a looser cap
//! lose less (down to ~23 m/s at 10 firings, 400 000 km), and the 1-a-year count
//! moves across the line inside that bracket.
//!
//! # The parking orbit's plane: pre-aimed, so only its tilt limits it
//! The departure asymptote has to lie in the parking orbit's plane on the departure
//! date. The plane turns with Earth's oblateness (about 7° a day at 400 km and
//! 28.5°), but a parked launch **knows its departure date when it launches**, and
//! the turning is predictable, so the launch's time of day - which sets where the
//! plane crosses the equator - is chosen so the plane has turned into the right
//! place on that day. That costs nothing. What cannot be aimed is the tilt: an
//! orbit launched due east from the Cape is tilted [`PARKING_INCLINATION_DEG`] =
//! 28.5°, and contains only asymptotes within that of the equator. So a window
//! whose asymptote is steeper is **not offered** to a parked launch
//! ([`parking_plane_reaches`]). (A steeper tilt would cost launch performance that
//! is not sourced here.) Holding the plane on schedule over a year means holding
//! the altitude - part of the station-keeping below.
//!
//! # The wait is charged (2026-10-10)
//! Drag make-up during the wait is priced by [`crate::station_keeping`]: NASA
//! TM-4527's density table (the whole solar cycle's median of the day-side
//! maximum), the stack filling Falcon's extended fairing and tumbling - both upper
//! bounds - and made up by Orion's auxiliary engines (310 s), not the OMS-E's
//! spare starts. At 400 km that is ~31 m/s a year, ~1 % of the stack: what is left
//! after a wait `t` is `e^(-k t)` ([`ParkedDelivery::wait_decay_per_s`]), and the
//! planner parks each launch as late as its chain allows
//! ([`crate::campaign::parked_launches`]). The plane trims stay free: the plane is
//! pre-aimed (above).
//!
//! # What this does not model
//! - **The Moon and Sun during the phasing loops**, which the apogee cap keeps
//!   small, and Earth's oblateness during them.
//! - **Rendezvous and docking** (two parked launches leaving the same date are
//!   counted as joined at no mass cost; by linearity they could equally leave as
//!   two stacks on that date).

use crate::departure_burn::FiniteBurn;
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

/// Parking-orbit altitude, m — a height a stack can be held at for the year and
/// more it waits (see the module doc). A stated choice, not a sourced figure.
pub const PARKING_ALTITUDE_M: f64 = 400_000.0;

/// The Falcon guide's baselined transfer-orbit perigee, m (§3.1) — the first cut's
/// parking orbit. A stack cannot wait there (it decays in weeks); bracket only.
pub const GUIDE_TRANSFER_PERIGEE_ALTITUDE_M: f64 = 185_000.0;

/// The parking orbit's tilt, degrees: a due-east launch from the Cape. It contains
/// only departure asymptotes within this angle of the equator.
pub const PARKING_INCLINATION_DEG: f64 = 28.5;

/// Whether a parking orbit tilted [`PARKING_INCLINATION_DEG`] can contain a
/// departure with hyperbolic excess velocity `v_inf_icrf` (ICRF, Earth's equator):
/// its declination must be within the tilt. The plane's turning is pre-aimed by the
/// launch time (module doc), so the tilt is the only limit.
pub fn parking_plane_reaches(v_inf_icrf: nalgebra::Vector3<f64>) -> bool {
    let n = v_inf_icrf.norm();
    n > 0.0 && (v_inf_icrf.z / n).asin().abs() <= PARKING_INCLINATION_DEG.to_radians()
}

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
    /// Vacuum thrust, N, its longest rated single firing, s, and its rated number
    /// of starts — the limits a finite escape burn is flown against
    /// ([`crate::departure_burn`]). `None` where they are not sourced here: such an
    /// engine can only be priced on the impulsive burn.
    pub limits: Option<EngineLimits>,
}

/// A departure engine's sourced firing limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineLimits {
    /// Vacuum thrust, N.
    pub thrust_n: f64,
    /// Longest rated single firing, s.
    pub max_firing_s: f64,
    /// Rated number of starts.
    pub max_starts: u32,
}

/// Orion's main engine (the Shuttle OMS engine, MMH / MON-3): nominal 315.1 s —
/// Belair et al., Space Propulsion 2024, SP2024_382, Table 2.
/// Its limits are the same table's: "Thrust 26.7 kN … Max Burn Duration 1030
/// seconds … Number of Starts 10 max".
pub const ORION_OMS_E: DepartureEngine = DepartureEngine {
    name: "OMS-E (storable)",
    isp_s: 315.1,
    storable: true,
    limits: Some(EngineLimits {
        thrust_n: 26_700.0,
        max_firing_s: 1_030.0,
        max_starts: 10,
    }),
};

/// RL10B-2 (liquid hydrogen / liquid oxygen): 465.5 s — National Research Council
/// (2006), Appendix D, p. 256. Hydrogen boils off; a *what if* only.
pub const RL10B_2: DepartureEngine = DepartureEngine {
    name: "RL10B-2 (hydrogen)",
    isp_s: 465.5,
    storable: false,
    limits: None,
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
    /// How the escape burn is flown.
    pub escape: EscapeBurn,
    /// What waiting in the parking orbit costs ([`crate::station_keeping`]), or
    /// `None` for a free wait (the model before 2026-10-10, kept for comparison).
    pub waiting: Option<crate::station_keeping::Waiting>,
}

/// How a parked stack's escape burn is priced.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EscapeBurn {
    /// All at once — the idealised burn, no loss. For an engine whose firing
    /// limits are not sourced, and for comparison.
    Impulsive,
    /// Flown finite, in `firings` firings at perigee within the engine's limits,
    /// no intermediate apogee beyond `apogee_cap_m` (m from Earth's centre) —
    /// [`crate::departure_burn`].
    Finite {
        /// Firings the escape uses (the engine's other starts are left in reserve).
        firings: u32,
        /// The highest apogee a phasing orbit may reach, m.
        apogee_cap_m: f64,
    },
}

impl ParkedDelivery {
    /// The escape burn from the parking orbit to characteristic energy `c3_km2_s2`
    /// (km²/s², the launch tables' unit), m/s — impulsive, from circular speed to
    /// the hyperbola's perigee speed.
    pub fn departure_dv_m_s(&self, c3_km2_s2: f64) -> f64 {
        crate::departure_burn::impulsive_dv_m_s(self.parking_radius_m(), c3_km2_s2)
    }

    /// The mass that leaves the parking orbit toward the rock, kg — the stack less
    /// the propellant the escape burn takes. The spent stage stays on, so this is
    /// the *separated* mass in the launch tables' sense. `0` for a non-finite `C3`.
    ///
    /// With a [`EscapeBurn::Finite`] escape, the burn's loss is added to the
    /// impulsive Δv (read off a table flown once per delivery and C3 knot, see
    /// [`escape_loss_m_s`](Self::escape_loss_m_s)); `0` where no schedule within
    /// the engine's limits reaches that `C3`.
    pub fn separated_mass_kg(&self, c3_km2_s2: f64) -> f64 {
        if !c3_km2_s2.is_finite() {
            return 0.0;
        }
        let Some(loss) = self.escape_loss_m_s(c3_km2_s2) else {
            return 0.0;
        };
        let dv = self.departure_dv_m_s(c3_km2_s2) + loss;
        self.stack_kg * (1.0 - propellant_fraction(dv, self.engine.isp_s))
    }

    /// The finite burn this delivery flies, or `None` for an impulsive escape.
    ///
    /// # Panics
    /// On a finite escape with an engine that has no sourced limits, or with more
    /// firings than the engine's rated starts — a delivery that cannot be flown.
    pub fn finite_burn(&self) -> Option<FiniteBurn> {
        let EscapeBurn::Finite {
            firings,
            apogee_cap_m,
        } = self.escape
        else {
            return None;
        };
        let lim = self.engine.limits.unwrap_or_else(|| {
            panic!("{}: a finite escape needs sourced engine limits", self.name)
        });
        assert!(
            (1..=lim.max_starts).contains(&firings),
            "{}: {firings} firings against {} rated starts",
            self.name,
            lim.max_starts
        );
        Some(FiniteBurn {
            thrust_n: lim.thrust_n,
            isp_s: self.engine.isp_s,
            max_firing_s: lim.max_firing_s,
            firings,
            apogee_cap_m,
        })
    }

    /// The parking orbit's radius, m.
    pub fn parking_radius_m(&self) -> f64 {
        crate::geometry::EARTH_EQUATORIAL_RADIUS_M + self.parking_altitude_m
    }

    /// The escape burn's loss against the impulsive burn, m/s: `0` for an
    /// impulsive escape, `None` where no finite schedule reaches `c3_km2_s2`.
    ///
    /// Between [`LOSS_TABLE_C3_STEP`] knots up to [`LOSS_TABLE_C3_MAX`] the loss is
    /// interpolated linearly from a table flown once per delivery (and cached for
    /// the process); outside it, or next to an unreachable knot, it is flown.
    pub fn escape_loss_m_s(&self, c3_km2_s2: f64) -> Option<f64> {
        let Some(burn) = self.finite_burn() else {
            return Some(0.0);
        };
        let fly = |c3: f64| {
            burn.fly(self.stack_kg, self.parking_radius_m(), c3)
                .map(|o| o.loss_m_s)
        };
        if !(0.0..LOSS_TABLE_C3_MAX).contains(&c3_km2_s2) {
            return fly(c3_km2_s2);
        }
        let x = c3_km2_s2 / LOSS_TABLE_C3_STEP;
        let i = x.floor() as usize;
        let table = loss_table(self);
        match (table[i], table[i + 1]) {
            (Some(a), Some(b)) => Some(a + (b - a) * (x - i as f64)),
            _ => fly(c3_km2_s2),
        }
    }

    /// The mass that hits the rock, kg: [`separated_mass_kg`](Self::separated_mass_kg)
    /// through the same cruise loss a direct launch takes.
    pub fn impact_mass_kg(&self, c3_km2_s2: f64) -> f64 {
        impact_mass_kg(self.separated_mass_kg(c3_km2_s2))
    }

    /// How fast waiting eats the stack, per second: what is left after `t` seconds
    /// in the parking orbit is `e^(−k t)` ([`crate::station_keeping::Waiting`]).
    /// `0` for a free wait. Priced at the full stack mass (see
    /// [`DragModel::mass_left`](crate::station_keeping::DragModel::mass_left)).
    ///
    /// # Panics
    /// If the parking altitude is outside the density table (350-1 000 km) - a wait
    /// that cannot be priced is not offered as free.
    pub fn wait_decay_per_s(&self) -> f64 {
        self.waiting.map_or(0.0, |w| {
            w.decay_per_s(self.parking_altitude_m, self.stack_kg)
                .unwrap_or_else(|| {
                    panic!(
                        "{}: no density for a {} km parking orbit",
                        self.name,
                        self.parking_altitude_m / 1e3
                    )
                })
        })
    }

    /// [`impact_mass_kg`](Self::impact_mass_kg) after `wait_s` in the parking orbit.
    pub fn impact_mass_after_wait_kg(&self, c3_km2_s2: f64, wait_s: f64) -> f64 {
        self.impact_mass_kg(c3_km2_s2) * (-self.wait_decay_per_s() * wait_s.max(0.0)).exp()
    }
}

/// The loss table's knot spacing in `C3`, km²/s². Linear between knots is good to
/// well under 0.1 m/s (pinned against flown midpoints).
pub const LOSS_TABLE_C3_STEP: f64 = 1.0;

/// The loss table's upper end, km²/s² — Falcon Heavy expendable's table ends at
/// `C3` 100, and no window above it is flown. Above it the loss is flown directly.
pub const LOSS_TABLE_C3_MAX: f64 = 100.0;

/// Every finite-escape loss table flown so far in this process, by delivery.
/// A handful at most (the shipping delivery and the bracket variants), so a list.
type LossTable = std::sync::Arc<Vec<Option<f64>>>;
static LOSS_TABLES: std::sync::Mutex<Vec<(ParkedDelivery, LossTable)>> =
    std::sync::Mutex::new(Vec::new());

/// `delivery`'s loss at every knot `0, STEP, …, MAX`, flown on first use (about a
/// tenth of a second) and held. The lock is held while flying, so two threads
/// asking at once fly it once.
fn loss_table(delivery: &ParkedDelivery) -> LossTable {
    let mut tables = LOSS_TABLES.lock().unwrap_or_else(|p| p.into_inner());
    if let Some((_, t)) = tables.iter().find(|(d, _)| d == delivery) {
        return t.clone();
    }
    let burn = delivery
        .finite_burn()
        .expect("a loss table is only asked for a finite escape");
    let knots = (LOSS_TABLE_C3_MAX / LOSS_TABLE_C3_STEP).round() as usize;
    let t: LossTable = std::sync::Arc::new(
        (0..=knots)
            .map(|k| {
                burn.fly(
                    delivery.stack_kg,
                    delivery.parking_radius_m(),
                    k as f64 * LOSS_TABLE_C3_STEP,
                )
                .map(|o| o.loss_m_s)
            })
            .collect(),
    );
    tables.push((*delivery, t.clone()));
    t
}

/// Firings the shipping escape burn uses, of the OMS-E's 10 rated starts - two
/// kept for trims during the wait. The user's choice from the bracket (module doc).
pub const SHIPPING_ESCAPE_FIRINGS: u32 = 8;

/// The highest apogee a shipping phasing orbit may reach, m: 100 000 km, where the
/// Moon's tidal pull is 0.04 % of Earth's ([`crate::departure_burn`]).
pub const SHIPPING_APOGEE_CAP_M: f64 = 1.0e8;

/// The shipping parked launch: the published single-payload limit, a storable
/// engine flown in 8 firings, a 400 km orbit.
pub const SHIPPING_PARKED_DELIVERY: ParkedDelivery = ParkedDelivery {
    name: "26.5 t stack, storable",
    stack_kg: FALCON_SINGLE_PAYLOAD_LIMIT_KG,
    engine: ORION_OMS_E,
    parking_altitude_m: PARKING_ALTITUDE_M,
    escape: EscapeBurn::Finite {
        firings: SHIPPING_ESCAPE_FIRINGS,
        apogee_cap_m: SHIPPING_APOGEE_CAP_M,
    },
    waiting: Some(crate::station_keeping::SHIPPING_WAITING),
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
        escape: EscapeBurn::Impulsive,
        waiting: Some(crate::station_keeping::SHIPPING_WAITING),
    },
    ParkedDelivery {
        name: "63.8 t stack, storable",
        stack_kg: FALCON_HEAVY_ADVERTISED_LEO_KG,
        engine: ORION_OMS_E,
        parking_altitude_m: PARKING_ALTITUDE_M,
        escape: EscapeBurn::Impulsive,
        waiting: Some(crate::station_keeping::SHIPPING_WAITING),
    },
    ParkedDelivery {
        name: "63.8 t stack, hydrogen",
        stack_kg: FALCON_HEAVY_ADVERTISED_LEO_KG,
        engine: RL10B_2,
        parking_altitude_m: PARKING_ALTITUDE_M,
        escape: EscapeBurn::Impulsive,
        waiting: Some(crate::station_keeping::SHIPPING_WAITING),
    },
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::launch_vehicle::FALCON_HEAVY_EXPENDABLE;

    /// The burn is the vis-viva difference: at `C3 = 0` it is exactly escape minus
    /// circular speed (~3.18 km/s from 400 km), and it grows with `C3`.
    #[test]
    fn the_escape_burn_is_vis_viva() {
        let d = SHIPPING_PARKED_DELIVERY;
        let r = crate::geometry::EARTH_EQUATORIAL_RADIUS_M + PARKING_ALTITUDE_M;
        let v_c = (EARTH_GM_M3_S2 / r).sqrt();
        let dv0 = d.departure_dv_m_s(0.0);
        assert!((dv0 - (2.0_f64.sqrt() - 1.0) * v_c).abs() < 1e-9, "{dv0}");
        assert!((3_150.0..3_200.0).contains(&dv0), "{dv0}");
        let mut last = dv0;
        for c3 in [1.0, 10.0, 40.0, 80.0] {
            let dv = d.departure_dv_m_s(c3);
            assert!(dv > last, "C3 {c3}: {dv} not above {last}");
            last = dv;
        }
        // From higher up the burn is smaller: the guide's 185 km would cost more.
        let low = ParkedDelivery {
            parking_altitude_m: GUIDE_TRANSFER_PERIGEE_ALTITUDE_M,
            ..d
        };
        let saved = low.departure_dv_m_s(10.0) - d.departure_dv_m_s(10.0);
        assert!((30.0..60.0).contains(&saved), "{saved}");
    }

    /// The rocket equation, to the digit, and the stage stays on: nothing but
    /// propellant leaves.
    #[test]
    fn the_mass_out_is_the_rocket_equation_with_the_stage_kept() {
        let d = SHIPPING_PARKED_DELIVERY;
        for c3 in [0.0, 12.0, 38.0] {
            let dv = d.departure_dv_m_s(c3) + d.escape_loss_m_s(c3).expect("reaches");
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

    /// The shipping escape is flown finite, within the OMS-E's rated starts, and
    /// it costs mass at every `C3`: an impulsive copy of it always carries more.
    #[test]
    fn the_shipping_escape_is_finite_and_costs_mass() {
        let s = SHIPPING_PARKED_DELIVERY;
        let burn = s.finite_burn().expect("the shipping escape is finite");
        assert!(burn.firings <= ORION_OMS_E.limits.unwrap().max_starts);
        let ideal = ParkedDelivery {
            escape: EscapeBurn::Impulsive,
            ..s
        };
        assert_eq!(ideal.escape_loss_m_s(43.0), Some(0.0));
        for c3 in [1.0, 20.0, 43.0, 62.7, 90.0] {
            let loss = s.escape_loss_m_s(c3).expect("reaches");
            assert!(loss > 0.0, "C3 {c3}: {loss}");
            assert!(
                s.separated_mass_kg(c3) < ideal.separated_mass_kg(c3),
                "C3 {c3}"
            );
        }
        // Every what-if is impulsive (no sourced limits for the hydrogen engine,
        // and one OMS-E is not a stage for 63.8 t), so it is labelled idealised.
        for w in WHAT_IF_PARKED_DELIVERIES {
            assert_eq!(w.escape, EscapeBurn::Impulsive, "{}", w.name);
        }
    }

    /// Linear between the table's knots is good to well under 0.1 m/s: the
    /// interpolated loss against the burn flown at the midpoints.
    #[test]
    fn the_loss_table_interpolates_the_flown_loss() {
        let s = SHIPPING_PARKED_DELIVERY;
        let burn = s.finite_burn().unwrap();
        let mut worst: f64 = 0.0;
        for c3 in [0.5, 10.5, 25.5, 38.5, 43.5, 62.5, 80.5, 99.5] {
            let flown = burn
                .fly(s.stack_kg, s.parking_radius_m(), c3)
                .expect("reaches")
                .loss_m_s;
            worst = worst.max((s.escape_loss_m_s(c3).unwrap() - flown).abs());
        }
        assert!(worst < 0.1, "{worst} m/s");
        // Past the table it is flown, not extrapolated.
        let past = burn
            .fly(s.stack_kg, s.parking_radius_m(), 104.0)
            .map(|o| o.loss_m_s);
        assert_eq!(s.escape_loss_m_s(104.0), past);
    }

    /// The plane rule: an asymptote within the parking orbit's 28.5° tilt is
    /// reachable, a steeper one (either side of the equator) is not.
    #[test]
    fn the_parking_plane_reaches_only_within_its_tilt() {
        use nalgebra::Vector3;
        let at = |deg: f64| {
            let d = deg.to_radians();
            Vector3::new(3_000.0 * d.cos(), 0.0, 3_000.0 * d.sin())
        };
        for deg in [0.0, 10.0, -27.2, 28.4, -28.4] {
            assert!(parking_plane_reaches(at(deg)), "{deg}");
        }
        for deg in [28.6, -28.6, 58.4, -90.0] {
            assert!(!parking_plane_reaches(at(deg)), "{deg}");
        }
        assert!(!parking_plane_reaches(Vector3::zeros()));
    }

    /// The shipping escape reaches every knot of its loss table. At high `C3` the
    /// phasing lengths that work are a sliver just under the apogee cap's edge; a
    /// first search scanned blindly, stepped over it above `C3` ~95 and priced
    /// those departures at zero mass.
    #[test]
    fn the_shipping_escape_reaches_the_whole_table() {
        let s = SHIPPING_PARKED_DELIVERY;
        let unreachable: Vec<usize> = loss_table(&s)
            .iter()
            .enumerate()
            .filter(|(_, l)| l.is_none())
            .map(|(k, _)| k)
            .collect();
        assert!(unreachable.is_empty(), "knots {unreachable:?}");
    }
}
