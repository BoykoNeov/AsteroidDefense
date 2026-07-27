//! Gravity tractor — the **gentle end** of §5's deflection spectrum (HANDOFF §5,
//! §6).
//!
//! A spacecraft station-keeps a fixed distance from the asteroid and simply
//! *hangs there*. Its own gravity tugs the asteroid toward it; the thrusters
//! cancel the asteroid's pull on the spacecraft so the pair does not collapse
//! together. Nothing touches the surface, so — as Lu & Love put it — the method
//! is "insensitive to the structure, surface properties, and rotation state of
//! the asteroid". That insensitivity is the whole appeal: a kinetic impactor's
//! `β` and a standoff burst's coupling efficiency are both properties of a rock
//! nobody has visited, while `G·m/d²` is Newton.
//!
//! # The model
//!
//! ```text
//! a_tow = G · m_sc / d²          (constant while station-keeping holds d)
//! a     = ± a_tow · t̂,           t̂ = ĥ × r̂,     inside the tow window only
//! ```
//!
//! Three things distinguish this from every other term in this directory.
//!
//! **It has a time window.** Every other force here is on for the whole
//! integration, because gravity and sunlight do not switch off. A tractor is a
//! *mission*: it arrives, tugs for a while, and leaves. [`TowWindow`] is the
//! parameter no existing term has, and the reason the deflection solver grew a
//! duration axis.
//!
//! **Its magnitude does not depend on heliocentric distance.** Yarkovsky and SRP
//! both fade as `(r₀/r)^d` because they are driven by sunlight. A tractor's
//! separation `d` is held fixed *by station-keeping* — that is what the thrusters
//! are for — so the tow is the same at aphelion as at perihelion. In the shared
//! [`secular oracle`](super::secular_oracle)'s parametrization this term is
//! exactly the `d = 0` case, which is why it needs no new validation machinery.
//!
//! **Its magnitude does not depend on the asteroid's mass.** The asteroid is a
//! test particle and `G·m_sc/d²` is the acceleration *it* feels; the rock's own
//! mass cancels out of its equation of motion exactly as it does for solar
//! gravity. This is not an approximation — it is why a tractor's Δv is
//! predictable for a body whose mass is poorly known. Asteroid mass enters this
//! module in exactly **one** place, and it is not the tow: see
//! [`HoverGeometry::station_keeping_thrust_n`].
//!
//! # Direction is a mission choice, like the standoff burst's
//! The spacecraft can station-keep ahead of the asteroid or behind it, so the tug
//! is prograde or retrograde at the operator's discretion
//! ([`TowDirection`]) — unlike a kinetic impactor, whose Δv direction is fixed by
//! the arrival geometry of the transfer. Modeled along-track (`ĥ × r̂`), matching
//! the headline curve's direction (§5, §7); a tug is not an impulse, so this is a
//! sustained transverse acceleration rather than a Δv vector.
//!
//! # Provenance
//! Lu, E. T. & Love, S. G., *Gravitational tractor for towing asteroids*, Nature
//! **438**, 177–178 (2005); preprint `astro-ph/0509595`. There is **no fitted
//! coefficient** to source here — the paper's own quoted rate,
//!
//! ```text
//! Δv = 4.2×10⁻³ · (m / 2×10⁴ kg) · (d / 100 m)⁻²   m/s per year of hovering
//! ```
//!
//! is `G·m/d²` times a year, and [`lu_love_2005_delta_v_per_year_matches_the_paper`]
//! checks ours against it. What the paper genuinely supplies that Newton does not
//! is the **configuration** (a 20-tonne spacecraft hovering at `d/r = 1.5` over a
//! 200 m, 2 g/cm³ body) and the **cant-angle bookkeeping** in
//! [`HoverGeometry`].
//!
//! [`lu_love_2005_delta_v_per_year_matches_the_paper`]: #
//!
//! [`FixedCentralBody`]: super::relativity::FixedCentralBody

use super::relativity::{CentralBodyState, FixedCentralBody};
use super::{ForceError, ForceModel, GRAVITATIONAL_CONSTANT};
use crate::epoch::Epoch;
use crate::state::StateVector;
use nalgebra::Vector3;

/// Which side of the asteroid the spacecraft station-keeps on, and therefore
/// which way the tug points along the orbit.
///
/// An enum rather than a signed magnitude because a "negative spacecraft mass" is
/// not a thing, and the sign convention is exactly the kind of detail that gets
/// silently inverted. [`Prograde`](Self::Prograde) tugs along the motion and
/// raises the semi-major axis; [`Retrograde`](Self::Retrograde) lowers it. Both
/// deflect — which one is preferable is a b-plane question, not a physics one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TowDirection {
    /// Spacecraft ahead of the asteroid: tug along `+t̂`, `da/dt > 0`.
    Prograde,
    /// Spacecraft behind the asteroid: tug along `−t̂`, `da/dt < 0`.
    Retrograde,
}

impl TowDirection {
    /// `+1` prograde, `−1` retrograde — the factor applied to the tow magnitude.
    pub fn sign(self) -> f64 {
        match self {
            TowDirection::Prograde => 1.0,
            TowDirection::Retrograde => -1.0,
        }
    }
}

/// The interval over which the tractor is on station, as TDB seconds past J2000.
///
/// **Half-open, `[start, end)`.** A zero-length window therefore contributes
/// exactly zero (rather than a single instantaneous sample of unclear weight),
/// and two back-to-back windows cannot double-count their shared instant. The
/// duration solver bisects on the length of one of these, so "zero duration means
/// zero deflection" has to be true by construction, not approximately.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TowWindow {
    start_s: f64,
    end_s: f64,
}

impl TowWindow {
    /// A window from `start` to `end`. `None` unless both epochs are finite and
    /// `end > start` — a reversed or degenerate window is a caller bug, not a
    /// silently-zero force.
    pub fn new(start: Epoch, end: Epoch) -> Option<Self> {
        let (start_s, end_s) = (start.tdb_seconds_past_j2000(), end.tdb_seconds_past_j2000());
        if !(start_s.is_finite() && end_s.is_finite() && end_s > start_s) {
            return None;
        }
        Some(Self { start_s, end_s })
    }

    /// A window of `duration_seconds` beginning at `start`. `None` for a
    /// non-positive or non-finite duration — the form the duration solver uses,
    /// so its bracket cannot accidentally probe a reversed window.
    pub fn from_duration(start: Epoch, duration_seconds: f64) -> Option<Self> {
        if !(duration_seconds.is_finite() && duration_seconds > 0.0) {
            return None;
        }
        Self::new(start, start.shifted_by_seconds(duration_seconds))
    }

    /// Whether the tractor is on station at `epoch` (half-open `[start, end)`).
    pub fn contains(&self, epoch: Epoch) -> bool {
        let t = epoch.tdb_seconds_past_j2000();
        t >= self.start_s && t < self.end_s
    }

    /// Length of the window, seconds (always > 0 by construction).
    pub fn duration_seconds(&self) -> f64 {
        self.end_s - self.start_s
    }

    /// First epoch the tractor is on station.
    pub fn start(&self) -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(self.start_s)
    }

    /// First epoch the tractor is **no longer** on station.
    pub fn end(&self) -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(self.end_s)
    }
}

/// The hovering geometry — everything about the tractor that is *not* the
/// resulting acceleration on the asteroid.
///
/// This type exists to keep two quantities apart that a single "efficiency"
/// factor would fatally merge:
///
/// - [`tow_acceleration`](Self::tow_acceleration) — what the asteroid feels.
///   `G·m_sc/d²`. Takes **no** asteroid mass and **no** cant angle.
/// - [`station_keeping_thrust_n`](Self::station_keeping_thrust_n) — what the
///   spacecraft must produce to stay there. Needs **both** the asteroid mass and
///   the cant angle.
///
/// Canting the thrusters outward (so the exhaust misses the surface) costs
/// *thrust*: the useful component is `cos(cant)` of what the engines make, so the
/// engines must be run harder. It does **not** reduce the spacecraft's gravity,
/// which does not care where the nozzles point. Lu & Love's own equation states
/// this — the cant appears on the left, with the thrust, and the gravitational
/// attraction on the right has no cant in it:
///
/// ```text
/// T·cos[sin⁻¹(r/d) + φ] = G·M·m/d²
/// ```
///
/// Putting a `cos(cant)` on the tow instead would understate the delivered Δv
/// while looking conservative — the same shape of bug as a `payload_kg` that
/// silently meant two different things. The split is enforced here by the
/// signatures: the tow method cannot see the cant angle it would need.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoverGeometry {
    /// Spacecraft mass, kg — the only thing that sets the tow.
    pub spacecraft_mass_kg: f64,
    /// Hover distance from the asteroid's **centre**, m (Lu & Love's `d`; their
    /// `d/r = 1.5` case is "one half radius above the surface").
    pub hover_distance_m: f64,
    /// Asteroid mean radius, m — needed for the plume-clearance geometry and the
    /// station-keeping thrust, never for the tow.
    pub asteroid_radius_m: f64,
    /// Exhaust plume half-width `φ`, radians. Lu & Love use 20°.
    pub plume_half_width_rad: f64,
}

impl HoverGeometry {
    /// Lu & Love 2005's notional configuration: a 20-tonne spacecraft hovering at
    /// `d/r = 1.5` over a 200 m-diameter asteroid, plume half-width 20°.
    ///
    /// Kept as a named constructor so the paper's case can be validated as its
    /// own row without any campaign-specific number leaking into it.
    pub fn lu_love_2005() -> Self {
        Self {
            spacecraft_mass_kg: 2.0e4,
            hover_distance_m: 150.0,
            asteroid_radius_m: 100.0,
            plume_half_width_rad: 20.0_f64.to_radians(),
        }
    }

    /// The closest a spacecraft with an exhaust plume of half-width `φ` can hover,
    /// in body radii, and still have a station-keeping solution — `1/cos φ`.
    ///
    /// # Why this is not `1` (and why a UI that assumes it is will lie)
    ///
    /// The cant is `sin⁻¹(r/d) + φ`, and the thrust divides by `cos` of it. So the
    /// wall is not the surface, it is wherever the cant reaches 90°:
    ///
    /// ```text
    ///   sin⁻¹(r/d) + φ = π/2   ⟺   r/d = cos φ   ⟺   d/r = 1/cos φ
    /// ```
    ///
    /// At Lu & Love's 20° plume that is **1.064 body radii**, not 1.0 — a band of
    /// hover distances that clear the surface, tow perfectly well, and have no
    /// station-keeping solution whatsoever. Approaching it the thrust diverges,
    /// which is the honest reason a tractor cannot simply hover closer to buy a
    /// larger `1/d²` tow; past it there is no station to keep.
    ///
    /// Exposed as a closed form so a control that offers a hover distance can take
    /// its lower bound from the physics instead of guessing a round number just
    /// above the surface. `None` for a plume that is not a sane half-angle
    /// (non-finite, negative, or ≥ 90°, which forbids hovering at any distance).
    pub fn min_hover_radii_for_station_keeping(plume_half_width_rad: f64) -> Option<f64> {
        if !plume_half_width_rad.is_finite()
            || plume_half_width_rad < 0.0
            || plume_half_width_rad >= std::f64::consts::FRAC_PI_2
        {
            return None;
        }
        Some(1.0 / plume_half_width_rad.cos())
    }

    /// The acceleration the asteroid feels, `G·m_sc/d²` (m/s²).
    ///
    /// Note what is absent: the asteroid's mass (it is a test particle) and the
    /// cant angle (gravity does not know about nozzles). `None` for a
    /// non-positive or non-finite mass or distance.
    pub fn tow_acceleration(&self) -> Option<f64> {
        let ok = self.spacecraft_mass_kg.is_finite()
            && self.spacecraft_mass_kg > 0.0
            && self.hover_distance_m.is_finite()
            && self.hover_distance_m > 0.0;
        if !ok {
            return None;
        }
        Some(GRAVITATIONAL_CONSTANT * self.spacecraft_mass_kg
            / (self.hover_distance_m * self.hover_distance_m))
    }

    /// The angle the thrusters must be tilted outward from the tow axis to keep
    /// the exhaust off the surface: `sin⁻¹(r/d) + φ` (radians).
    ///
    /// `None` if the spacecraft is not clear of the surface (`d ≤ r`), where the
    /// geometry is meaningless.
    pub fn cant_angle_rad(&self) -> Option<f64> {
        if !self.is_clear_of_surface() || !self.plume_half_width_rad.is_finite() {
            return None;
        }
        Some((self.asteroid_radius_m / self.hover_distance_m).asin() + self.plume_half_width_rad)
    }

    /// Whether the hover point is outside the body at all (`d > r`). A tractor
    /// inside the asteroid is not a conservative estimate, it is nonsense.
    ///
    /// **This is the weaker of the two geometric constraints**, and on its own it
    /// is not enough to call a configuration flyable — see
    /// [`Self::can_hold_station`]. Clearing the surface says the *tow* is
    /// meaningful; it says nothing about whether the spacecraft can stay there.
    pub fn is_clear_of_surface(&self) -> bool {
        self.hover_distance_m.is_finite()
            && self.asteroid_radius_m.is_finite()
            && self.asteroid_radius_m > 0.0
            && self.hover_distance_m > self.asteroid_radius_m
    }

    /// Whether a station-keeping solution exists at all: the cant must stay under
    /// 90°, or no amount of thrust has a component along the tow axis.
    ///
    /// Strictly stronger than [`Self::is_clear_of_surface`], and the distinction
    /// is a real one rather than defensive coding. Gravity does not care where the
    /// nozzles point, so a spacecraft hovering between
    /// [`Self::min_hover_radii_for_station_keeping`] and the surface still *tows* — the
    /// mission is what becomes impossible, not the physics. A readout that
    /// conflated the two would print a healthy tow beside a thrust of zero.
    pub fn can_hold_station(&self) -> bool {
        self.cant_angle_rad()
            .is_some_and(|cant| cant.cos().is_finite() && cant.cos() > 0.0)
    }

    /// Total thrust (N) the spacecraft must sustain to hold station over a body of
    /// `asteroid_mass_kg`:
    ///
    /// ```text
    /// T = G·M·m / (d² · cos[sin⁻¹(r/d) + φ])
    /// ```
    ///
    /// **This is the one place the asteroid's mass legitimately enters**, and it
    /// is a feasibility question, not a deflection one: it decides whether the
    /// mission can be flown, never how much Δv it delivers. A heavier rock needs
    /// a harder-working spacecraft to hover over it, but tows no more slowly.
    ///
    /// `None` if the geometry is degenerate or the cant reaches 90°, where no
    /// amount of thrust has a useful component along the tow axis — a real
    /// constraint that bites when the spacecraft hovers close (large `sin⁻¹(r/d)`)
    /// with a wide plume.
    pub fn station_keeping_thrust_n(&self, asteroid_mass_kg: f64) -> Option<f64> {
        if !(asteroid_mass_kg.is_finite() && asteroid_mass_kg > 0.0) {
            return None;
        }
        let cant = self.cant_angle_rad()?;
        let cos_cant = cant.cos();
        if !(cos_cant.is_finite() && cos_cant > 0.0) {
            return None;
        }
        let mutual = GRAVITATIONAL_CONSTANT * asteroid_mass_kg * self.spacecraft_mass_kg
            / (self.hover_distance_m * self.hover_distance_m);
        Some(mutual / cos_cant)
    }
}

/// A gravity tractor as a **windowed constant transverse acceleration** (§5).
///
/// Holds the signed tow magnitude, the window it is on station for, and a
/// [`CentralBodyState`] source for the Sun (the along-track direction is defined
/// by the body's motion *relative to the Sun*, exactly as in
/// [`super::yarkovsky::YarkovskyA2`]). Outside the window the term contributes
/// exactly zero.
pub struct GravityTractor {
    /// Signed tow acceleration (m/s²): `±G·m_sc/d²`, sign from [`TowDirection`].
    a_tow_signed: f64,
    /// When the spacecraft is on station.
    window: TowWindow,
    /// The central body (Sun) whose motion defines the heliocentric frame.
    central: Box<dyn CentralBodyState>,
}

impl GravityTractor {
    /// Build from an explicit **unsigned** tow magnitude plus a direction.
    pub fn new(
        tow_acceleration_ms2: f64,
        direction: TowDirection,
        window: TowWindow,
        central: impl CentralBodyState + 'static,
    ) -> Self {
        Self {
            a_tow_signed: direction.sign() * tow_acceleration_ms2,
            window,
            central: Box::new(central),
        }
    }

    /// Build from the physical hovering configuration — the constructor that
    /// carries the `G·m_sc/d²` coupling. `None` if [`HoverGeometry`] is
    /// degenerate.
    ///
    /// Deliberately does **not** consult the station-keeping thrust: a
    /// configuration whose thrust requirement is implausible still tows at
    /// exactly this rate, and conflating "infeasible mission" with "weaker tug"
    /// would quietly corrupt the physics. Feasibility is reported alongside, not
    /// folded in.
    pub fn hovering(
        hover: HoverGeometry,
        direction: TowDirection,
        window: TowWindow,
        central: impl CentralBodyState + 'static,
    ) -> Option<Self> {
        let a_tow = hover.tow_acceleration()?;
        Some(Self::new(a_tow, direction, window, central))
    }

    /// Kernel-free configuration for the isolation tests: Sun pinned at the frame
    /// origin at rest.
    pub fn sun_at_origin(
        tow_acceleration_ms2: f64,
        direction: TowDirection,
        window: TowWindow,
    ) -> Self {
        Self::new(
            tow_acceleration_ms2,
            direction,
            window,
            FixedCentralBody::at_rest_origin(),
        )
    }

    /// The signed tow acceleration (m/s²) — exposed so a caller can quote the
    /// **impulsive-equivalent** Δv of a full window as `|a_tow| · duration`
    /// without re-deriving `G·m/d²`.
    pub fn tow_acceleration_signed(&self) -> f64 {
        self.a_tow_signed
    }

    /// The window this tractor is on station for.
    pub fn window(&self) -> TowWindow {
        self.window
    }
}

impl ForceModel for GravityTractor {
    fn acceleration(&self, epoch: Epoch, state: &StateVector) -> Result<Vector3<f64>, ForceError> {
        // Off station: contribute exactly zero, and — importantly — do *not*
        // consult the ephemeris or the state. A tractor that is not there cannot
        // fail, so a window outside the propagation's valid range is not an error.
        if !self.window.contains(epoch) {
            return Ok(Vector3::zeros());
        }

        let sun = self.central.state_at(epoch)?;
        let r = state.position - sun.position;
        let v = state.velocity - sun.velocity;

        let r_norm = r.norm();
        if r_norm == 0.0 || !r_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: r_norm,
            });
        }

        // Transverse (prograde, in-plane) unit vector t̂ = ĥ × r̂ — the same
        // construction as the Yarkovsky term, and for the same reason: off-apsis
        // this is *not* v̂, and picking v̂ is the classic error.
        let h = r.cross(&v);
        let h_norm = h.norm();
        if h_norm == 0.0 || !h_norm.is_finite() {
            return Err(ForceError::Singularity {
                perturber_index: 0,
                separation: h_norm,
            });
        }
        let r_hat = r / r_norm;
        let h_hat = h / h_norm;
        let t_hat = h_hat.cross(&r_hat);

        // No (r₀/r)^d factor: station-keeping holds the separation fixed, so the
        // tow does not fade with heliocentric distance (see the module docs).
        Ok(self.a_tow_signed * t_hat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forces::point_mass::{FixedPerturber, PointMassGravity};
    use crate::forces::secular_oracle::{
        closed_form_constant_circular, osculating_a, slope_per_step, time_averaged,
    };
    use crate::forces::CompositeForce;
    use crate::integrator::{Dop853, Integrator};

    const MU_SUN: f64 = 1.327_124_400_18e20;
    const AU: f64 = 1.495_978_707e11;
    /// Julian year, seconds — the "per year of hovering" unit Lu & Love quote in.
    const JULIAN_YEAR_S: f64 = 365.25 * 86_400.0;

    fn epoch0() -> Epoch {
        Epoch::from_tdb_seconds_past_j2000(0.0)
    }

    fn window(start_s: f64, end_s: f64) -> TowWindow {
        TowWindow::new(
            Epoch::from_tdb_seconds_past_j2000(start_s),
            Epoch::from_tdb_seconds_past_j2000(end_s),
        )
        .expect("valid window")
    }

    /// An always-on window, for the tests that are about the acceleration rather
    /// than the schedule.
    fn always() -> TowWindow {
        window(-1.0e12, 1.0e12)
    }

    // ---------------------------------------------------------------- the tow

    /// The published anchor, and the reason this term needs no fitted
    /// coefficient. Lu & Love 2005 quote the tow rate as
    ///
    /// ```text
    /// Δv = 4.2×10⁻³ · (m / 2×10⁴ kg) · (d / 100 m)⁻²   m/s per year
    /// ```
    ///
    /// Our `G·m/d²` over a Julian year must reproduce it. Checked at the paper's
    /// own configuration (`m = 20 t`, `d = 150 m`) *and* at a scaled one, so the
    /// test pins the `1/d²` shape rather than a single lucky point.
    ///
    /// The 1 % band is set by the paper's two-significant-figure coefficient
    /// (4.2 vs the 4.212 that `G·2×10⁴/(100 m)²·yr` actually gives = 0.28 %), not
    /// chosen to make the test pass.
    #[test]
    fn lu_love_2005_delta_v_per_year_matches_the_paper() {
        /// The paper's quoted rate, m/s per year of hovering.
        fn paper_dv_per_year(mass_kg: f64, d_m: f64) -> f64 {
            4.2e-3 * (mass_kg / 2.0e4) * (d_m / 100.0).powi(-2)
        }

        for (mass_kg, d_m) in [
            (2.0e4, 150.0), // the paper's own case: 20 t at d/r = 1.5 over a 200 m body
            (2.0e4, 100.0), // the coefficient's own reference distance
            (5.0e4, 225.0), // a scaled case, to pin the 1/d² shape
        ] {
            let hover = HoverGeometry {
                spacecraft_mass_kg: mass_kg,
                hover_distance_m: d_m,
                ..HoverGeometry::lu_love_2005()
            };
            let ours = hover.tow_acceleration().unwrap() * JULIAN_YEAR_S;
            let paper = paper_dv_per_year(mass_kg, d_m);
            let rel = (ours - paper).abs() / paper;
            assert!(
                rel < 0.01,
                "m={mass_kg} kg d={d_m} m: ours {ours:.4e} m/s/yr vs paper {paper:.4e} (rel {rel:.4})"
            );
        }
    }

    /// The other published anchor: at Lu & Love's configuration the spacecraft
    /// must "maintain a total thrust T = 1 N". Reproducing that from
    /// `G·M·m/(d²·cos[sin⁻¹(r/d)+φ])` validates the cant bookkeeping *and* the
    /// asteroid-mass path in one number.
    ///
    /// The asteroid mass is built from the paper's stated `r = 100 m` and
    /// `ρ = 2 g/cm³` rather than quoted, since the paper gives the density.
    #[test]
    fn lu_love_2005_station_keeping_thrust_is_about_one_newton() {
        let hover = HoverGeometry::lu_love_2005();
        let r = hover.asteroid_radius_m;
        let asteroid_mass = 4.0 / 3.0 * std::f64::consts::PI * r * r * r * 2000.0;

        // Sanity on the intermediate the paper also states parametrically:
        // G·M·m/d² = 1.12·(ρ/2)·(r/d)³·(m/2e4)·(d/100) N.
        let mutual = GRAVITATIONAL_CONSTANT * asteroid_mass * hover.spacecraft_mass_kg
            / (hover.hover_distance_m * hover.hover_distance_m);
        let paper_mutual = 1.12 * (r / hover.hover_distance_m).powi(3) * (hover.hover_distance_m / 100.0);
        assert!(
            (mutual - paper_mutual).abs() / paper_mutual < 0.01,
            "mutual attraction {mutual:.4} N vs the paper's parametrized {paper_mutual:.4} N"
        );

        let thrust = hover.station_keeping_thrust_n(asteroid_mass).unwrap();
        assert!(
            (thrust - 1.0).abs() < 0.1,
            "thrust {thrust:.3} N should be the paper's ~1 N"
        );
        // And the cant is what makes it 1 N rather than 0.5 N — worth pinning, so
        // a later edit cannot drop the cos() and still pass on a loose band.
        assert!(
            thrust > 2.0 * mutual - 0.1 * mutual,
            "canting must roughly double the required thrust: {thrust:.3} vs mutual {mutual:.3}"
        );
    }

    /// **The wall is not the surface.** Between `d/r = 1` and `d/r = 1/cos φ`
    /// there is a band where the spacecraft is outside the body, tows perfectly
    /// well, and has no station-keeping solution at all — the cant has reached 90°
    /// and no thrust direction has a component along the tow axis.
    ///
    /// This exists because that band is where a hover-distance control naturally
    /// puts its minimum. "Just above the surface" sounds like the safe bound and
    /// is not: at Lu & Love's 20° plume the real floor is 1.064 radii, and a
    /// control offering 1.02 lets a user reach a configuration whose thrust is
    /// `None` while every other number on screen stays healthy.
    #[test]
    fn station_keeping_fails_before_the_surface_does() {
        let phi = HoverGeometry::lu_love_2005().plume_half_width_rad;
        let floor = HoverGeometry::min_hover_radii_for_station_keeping(phi).expect("sane plume");
        // 1/cos(20°) = 1.0642, comfortably above the surface at 1.0.
        assert!(
            (floor - 1.0 / phi.cos()).abs() < 1.0e-15,
            "the floor must be exactly 1/cos(phi)"
        );
        assert!(
            floor > 1.06 && floor < 1.07,
            "at a 20 deg plume the floor should be ~1.064 radii, got {floor:.4}"
        );

        let at = |radii: f64| HoverGeometry {
            spacecraft_mass_kg: 2.0e4,
            hover_distance_m: radii * 100.0,
            asteroid_radius_m: 100.0,
            plume_half_width_rad: phi,
        };

        // Inside the band: clear of the surface, tows, cannot hold station.
        let doomed = at(1.02);
        assert!(doomed.is_clear_of_surface(), "1.02 radii is outside the body");
        assert!(
            doomed.tow_acceleration().is_some_and(|a| a > 0.0),
            "gravity does not care where the nozzles point — the tow is real here"
        );
        assert!(!doomed.can_hold_station(), "but the cant has passed 90 deg");
        assert!(
            doomed.station_keeping_thrust_n(1.0e11).is_none(),
            "and there is no thrust that holds it"
        );

        // Just outside the floor: everything defined, and the thrust is enormous —
        // the divergence is the honest reason you cannot hover closer for a bigger
        // 1/d^2 tow.
        let tight = at(floor * 1.001);
        assert!(tight.can_hold_station());
        let near_wall = tight.station_keeping_thrust_n(1.0e11).expect("defined");
        let comfortable = at(1.5).station_keeping_thrust_n(1.0e11).expect("defined");
        assert!(
            near_wall > 10.0 * comfortable,
            "thrust must blow up approaching the floor: {near_wall:.3} N vs \
             {comfortable:.3} N at d/r = 1.5"
        );

        // A plume that cannot be flown at any distance is rejected rather than
        // returning a floor a caller would treat as reachable.
        assert!(HoverGeometry::min_hover_radii_for_station_keeping(std::f64::consts::FRAC_PI_2).is_none());
        assert!(HoverGeometry::min_hover_radii_for_station_keeping(f64::NAN).is_none());
        assert!(HoverGeometry::min_hover_radii_for_station_keeping(-0.1).is_none());
        // A zero-width plume needs no cant beyond the surface tangent, so its floor
        // is exactly the surface.
        assert_eq!(HoverGeometry::min_hover_radii_for_station_keeping(0.0), Some(1.0));
    }

    /// **The bug this term is most likely to grow.** Canting the thrusters is a
    /// propellant cost, not a weaker tug: `G·m/d²` is indifferent to where the
    /// nozzles point. So changing `φ` must move the required thrust and leave the
    /// tow **bit-for-bit** identical.
    ///
    /// A `cos(cant)` factor wrongly applied to the tow would look conservative and
    /// silently understate every delivered Δv in the project — the same shape as
    /// the shipped `payload_kg`-means-two-things defect.
    #[test]
    fn cant_angle_changes_the_thrust_but_never_the_tow() {
        let base = HoverGeometry::lu_love_2005();
        let wide = HoverGeometry {
            plume_half_width_rad: 40.0_f64.to_radians(),
            ..base
        };
        let none = HoverGeometry {
            plume_half_width_rad: 0.0,
            ..base
        };
        let m_ast = 8.3776e9;

        assert_eq!(
            base.tow_acceleration().unwrap(),
            wide.tow_acceleration().unwrap(),
            "plume width must not touch the tow"
        );
        assert_eq!(
            base.tow_acceleration().unwrap(),
            none.tow_acceleration().unwrap(),
            "plume width must not touch the tow"
        );

        let (t_none, t_base, t_wide) = (
            none.station_keeping_thrust_n(m_ast).unwrap(),
            base.station_keeping_thrust_n(m_ast).unwrap(),
            wide.station_keeping_thrust_n(m_ast).unwrap(),
        );
        assert!(
            t_none < t_base && t_base < t_wide,
            "wider plume ⇒ more cant ⇒ more thrust: {t_none:.3} < {t_base:.3} < {t_wide:.3} N"
        );
    }

    /// Asteroid mass must not reach the tow at all — the test-particle property
    /// that makes a tractor's Δv predictable for a body of unknown mass. Two rocks
    /// differing by 10³ in mass are towed identically.
    #[test]
    fn asteroid_mass_changes_the_thrust_but_never_the_tow() {
        let hover = HoverGeometry::lu_love_2005();
        let light = hover.station_keeping_thrust_n(1.0e7).unwrap();
        let heavy = hover.station_keeping_thrust_n(1.0e10).unwrap();
        assert!(heavy > 100.0 * light, "thrust must scale with asteroid mass");
        // The tow method cannot even accept a mass — this is enforced by the
        // signature, so all that is left to check is that it is a pure function
        // of the spacecraft configuration.
        assert_eq!(
            hover.tow_acceleration().unwrap(),
            HoverGeometry::lu_love_2005().tow_acceleration().unwrap()
        );
    }

    /// Degenerate hovering geometry fails loud rather than returning a plausible
    /// number: inside the body, on the surface, or canted past 90° where no
    /// thrust direction can hold station.
    #[test]
    fn degenerate_hover_geometry_is_rejected() {
        let inside = HoverGeometry {
            hover_distance_m: 80.0,
            ..HoverGeometry::lu_love_2005()
        };
        assert!(!inside.is_clear_of_surface());
        assert!(inside.cant_angle_rad().is_none());
        assert!(inside.station_keeping_thrust_n(8.4e9).is_none());
        // ...but the tow itself is still well-defined arithmetic; it is the
        // *mission* that is impossible, not `G·m/d²`.
        assert!(inside.tow_acceleration().is_some());

        // Hovering very close with a wide plume drives the cant past 90°.
        let over_canted = HoverGeometry {
            hover_distance_m: 101.0,
            plume_half_width_rad: 45.0_f64.to_radians(),
            ..HoverGeometry::lu_love_2005()
        };
        assert!(over_canted.cant_angle_rad().unwrap() > std::f64::consts::FRAC_PI_2);
        assert!(over_canted.station_keeping_thrust_n(8.4e9).is_none());

        assert!(HoverGeometry {
            spacecraft_mass_kg: -1.0,
            ..HoverGeometry::lu_love_2005()
        }
        .tow_acceleration()
        .is_none());
    }

    // ------------------------------------------------------------- direction

    /// The acceleration is transverse (`a·r̂ = 0`), of magnitude `G·m/d²`, along
    /// `ĥ×r̂` — which off-apsis is **not** `v̂`. Same trap as Yarkovsky's, checked
    /// the same way, because it is the same construction.
    #[test]
    fn acceleration_is_transverse_not_along_velocity() {
        let a_tow = 2.6e-11;
        let rx = 0.7 * AU;
        let s = StateVector::from_components(rx, 0.0, 0.0, 5_000.0, 30_000.0, 0.0);
        let a = GravityTractor::sun_at_origin(a_tow, TowDirection::Prograde, always())
            .acceleration(epoch0(), &s)
            .unwrap();

        assert!(a.x.abs() < 1e-26, "must be perpendicular to r: {a:?}");
        assert!(a.z.abs() < 1e-26, "planar motion stays planar: {a:?}");
        assert!(
            (a.y - a_tow).abs() < 1e-6 * a_tow,
            "a.y={} expected the un-scaled tow {a_tow}",
            a.y
        );
        let cos_with_v = a.dot(&s.velocity) / (a.norm() * s.velocity.norm());
        assert!(cos_with_v < 0.99, "must not be along v̂ (cos={cos_with_v})");
    }

    /// Retrograde station-keeping flips the tug, and nothing else.
    #[test]
    fn retrograde_tug_is_the_exact_negative_of_prograde() {
        let s = StateVector::from_components(0.8 * AU, 0.1 * AU, 0.0, -3_000.0, 28_000.0, 0.0);
        let pro = GravityTractor::sun_at_origin(2.6e-11, TowDirection::Prograde, always())
            .acceleration(epoch0(), &s)
            .unwrap();
        let retro = GravityTractor::sun_at_origin(2.6e-11, TowDirection::Retrograde, always())
            .acceleration(epoch0(), &s)
            .unwrap();
        assert_eq!(pro, -retro);
    }

    /// The tow does **not** fade with heliocentric distance — the property that
    /// separates it from Yarkovsky and SRP. Same magnitude at 0.7 AU and 3 AU.
    #[test]
    fn tow_does_not_fade_with_heliocentric_distance() {
        let term = GravityTractor::sun_at_origin(2.6e-11, TowDirection::Prograde, always());
        let near = StateVector::from_components(0.7 * AU, 0.0, 0.0, 0.0, 35_000.0, 0.0);
        let far = StateVector::from_components(3.0 * AU, 0.0, 0.0, 0.0, 17_000.0, 0.0);
        let a_near = term.acceleration(epoch0(), &near).unwrap().norm();
        let a_far = term.acceleration(epoch0(), &far).unwrap().norm();
        assert_eq!(
            a_near, a_far,
            "station-keeping holds d fixed, so the tow must not scale with r"
        );
    }

    #[test]
    fn degenerate_states_fail_loud() {
        let term = GravityTractor::sun_at_origin(2.6e-11, TowDirection::Prograde, always());
        let on_sun = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(matches!(
            term.acceleration(epoch0(), &on_sun),
            Err(ForceError::Singularity { .. })
        ));
        let radial = StateVector::from_components(AU, 0.0, 0.0, 1_000.0, 0.0, 0.0);
        assert!(matches!(
            term.acceleration(epoch0(), &radial),
            Err(ForceError::Singularity { .. })
        ));
    }

    /// A degenerate state *outside* the window must still be fine — the tractor
    /// is not there, so it cannot object. This matters in practice: a tow window
    /// covering part of a long arc must not make the rest of the arc fallible.
    #[test]
    fn off_station_never_fails_even_on_a_degenerate_state() {
        let term =
            GravityTractor::sun_at_origin(2.6e-11, TowDirection::Prograde, window(100.0, 200.0));
        let on_sun = StateVector::from_components(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        let outside = Epoch::from_tdb_seconds_past_j2000(50.0);
        assert_eq!(term.acceleration(outside, &on_sun).unwrap(), Vector3::zeros());
    }

    // ---------------------------------------------------------------- window

    /// The window is half-open `[start, end)`: on at the opening instant, off at
    /// the closing one. Pinned because the duration solver bisects on the length,
    /// and "zero duration ⇒ zero force" has to be exact.
    #[test]
    fn window_is_half_open() {
        let w = window(100.0, 200.0);
        let s = StateVector::from_components(AU, 0.0, 0.0, 0.0, 29_800.0, 0.0);
        let term = GravityTractor::sun_at_origin(2.6e-11, TowDirection::Prograde, w);
        let at = |t: f64| {
            term.acceleration(Epoch::from_tdb_seconds_past_j2000(t), &s)
                .unwrap()
                .norm()
        };
        assert_eq!(at(99.0), 0.0, "before the window");
        assert!(at(100.0) > 0.0, "the opening instant is inside");
        assert!(at(150.0) > 0.0, "mid-window");
        assert_eq!(at(200.0), 0.0, "the closing instant is outside");
        assert_eq!(at(201.0), 0.0, "after the window");

        assert!(w.contains(w.start()));
        assert!(!w.contains(w.end()));
        assert_eq!(w.duration_seconds(), 100.0);
    }

    /// Reversed, empty, and degenerate windows are constructor errors, not
    /// silently-zero forces.
    ///
    /// A non-finite *epoch* is deliberately not exercised: `Epoch` cannot hold
    /// one — `Epoch::from_tdb_seconds_past_j2000(f64::NAN)` panics inside
    /// hifitime's `Duration`, so the case is unreachable through this API rather
    /// than merely untested. The `is_finite` guard in `TowWindow::new` stays as
    /// defence against a future `Epoch` that is more permissive, but it is not a
    /// live code path today.
    #[test]
    fn invalid_windows_are_rejected() {
        let e = |t: f64| Epoch::from_tdb_seconds_past_j2000(t);
        assert!(TowWindow::new(e(200.0), e(100.0)).is_none(), "reversed");
        assert!(TowWindow::new(e(100.0), e(100.0)).is_none(), "empty");
        assert!(TowWindow::from_duration(e(0.0), 0.0).is_none(), "zero duration");
        assert!(TowWindow::from_duration(e(0.0), -5.0).is_none(), "negative");
        assert!(TowWindow::from_duration(e(0.0), f64::NAN).is_none(), "NaN duration");
        assert_eq!(
            TowWindow::from_duration(e(10.0), 40.0).unwrap(),
            TowWindow::new(e(10.0), e(50.0)).unwrap()
        );
    }

    /// Deliver a tow window to a **free** particle (no gravity at all) and return
    /// the relative error of the accumulated Δv against `a·T`.
    ///
    /// A free particle isolates the window from orbital dynamics: any discrepancy
    /// is the integrator's handling of the window edges and nothing else. The
    /// particle starts at 1 AU on `+x` drifting slowly along `+y`, so `t̂ = ĥ×r̂`
    /// is `+ŷ` and the tug is a constant push along one axis.
    fn window_delivery_rel_error(
        edges: (f64, f64),
        stepper: Dop853,
        n_steps: usize,
        total_s: f64,
    ) -> f64 {
        let a_tow = 2.6e-11;
        let (t_start, t_end) = edges;
        let term =
            GravityTractor::sun_at_origin(a_tow, TowDirection::Prograde, window(t_start, t_end));
        let s0 = StateVector::from_components(AU, 0.0, 0.0, 0.0, 1.0, 0.0);
        let model = CompositeForce::new().with(Box::new(term));

        let h = total_s / n_steps as f64;
        let mut state = s0;
        let mut epoch = epoch0();
        for _ in 0..n_steps {
            state = stepper.step(&model, epoch, &state, h).unwrap();
            epoch = epoch.shifted_by_seconds(h);
        }
        let delivered = state.velocity.y - s0.velocity.y;
        let expected = a_tow * (t_end - t_start);
        (delivered - expected) / expected
    }

    /// **The window-edge check, and the evidence behind not snapping edges to the
    /// snapshot cadence.**
    ///
    /// A hard on/off edge is a discontinuity in the derivative, generally landing
    /// *inside* whatever sub-step the adaptive driver is taking. The concern is
    /// that an embedded error estimator comparing polynomial fits across a
    /// discontinuity smears the edge and silently mis-delivers Δv. Measured
    /// rather than assumed, and the measurement is more specific than the concern:
    ///
    /// ```text
    ///                        rtol/atol 1e-9 (shipping)   rtol 1e-13 / atol 1e-6 (loose)
    ///   edges inside steps           −1.0e-4                        +6.3e-3
    ///   edges on boundaries          −9.2e-10                       −9.2e-10
    /// ```
    ///
    /// Aligning an edge to a step boundary is accurate at *any* tolerance, because
    /// then no step contains a discontinuity and the integrand is smooth
    /// everywhere it is sampled. A free edge is accurate only to the extent the
    /// error controller is asked to be: the discontinuity does not defeat the
    /// controller, it **converts a tolerance into a systematic Δv error**. Five
    /// orders of magnitude separate the two rows at the same tolerance, so it is
    /// alignment — not tolerance — that is doing the work.
    ///
    /// **Why the solver still leaves edges free.** `−1.0e-4` relative is the
    /// pessimistic end: this setup deliberately uses six enormous 2×10⁶ s outer
    /// steps and no gravity, so an edge-straddling step is a large fraction of the
    /// whole window. The real propagation runs at the snapshot cadence under full
    /// gravity, where the driver sub-steps far more finely and the straddling
    /// fraction is much smaller. Even taking `1e-4` at face value, on the
    /// campaign's ~10⁻² m/s tow that is ~10⁻⁶ m/s — orders below what the b-plane
    /// resolves. Snapping window edges to the cadence would buy that back at the
    /// cost of **quantizing the duration bisection to the cadence**, which is a
    /// worse trade. Recorded so the choice is a decision, not an accident.
    #[test]
    fn window_edges_are_accurate_at_shipping_tolerances() {
        let (total, n) = (1.2e7, 6);
        // Deliberately ugly edges — no chance of aligning with a boundary by luck.
        let free = (1.234_567e6, 9.876_543e6);
        // What matters here is only that both edges fall on multiples of the outer
        // step size h = 2e6 s; the window length need not match the free case.
        let aligned = (2.0e6, 1.0e7);

        let shipping = Dop853::new();
        let rel_free = window_delivery_rel_error(free, shipping, n, total);
        let rel_aligned = window_delivery_rel_error(aligned, shipping, n, total);
        assert!(
            rel_free.abs() < 1e-3,
            "a free window edge must stay well below the b-plane's resolution; got {rel_free:.3e}"
        );
        assert!(
            rel_aligned.abs() < 1e-8,
            "a boundary-aligned edge must be essentially exact; got {rel_aligned:.3e}"
        );
        assert!(
            rel_aligned.abs() < 1e-4 * rel_free.abs(),
            "alignment must dominate: aligned {rel_aligned:.3e} vs free {rel_free:.3e}"
        );

        // The loose-tolerance row is what makes this a measurement rather than an
        // assumption: slacken atol and the free edge degrades by ~60×, while the
        // aligned edge does not move at all.
        let loose = Dop853::new().with_tolerances(1e-13, 1e-6);
        let rel_loose_free = window_delivery_rel_error(free, loose, n, total);
        let rel_loose_aligned = window_delivery_rel_error(aligned, loose, n, total);
        assert!(
            rel_loose_free.abs() > 1e-3,
            "a free edge must visibly degrade at loose atol; got {rel_loose_free:.3e}"
        );
        assert!(
            rel_loose_aligned.abs() < 1e-8,
            "an aligned edge must not degrade at all; got {rel_loose_aligned:.3e}"
        );
    }

    // --------------------------------------------------------------- secular

    /// Integrate under Newtonian gravity ± the tractor and measure the secular
    /// da/dt by **stroboscopic** sampling of the osculating semi-major axis (once
    /// per period, so the intra-orbit wiggle cancels). `tug = None` is the control.
    fn measure_secular_da_dt(
        tug: Option<(f64, TowDirection)>,
        a: f64,
        e: f64,
        n_orbits: usize,
    ) -> f64 {
        let r_peri = a * (1.0 - e);
        let v_peri = (MU_SUN * (2.0 / r_peri - 1.0 / a)).sqrt();
        let mut state = StateVector::from_components(r_peri, 0.0, 0.0, 0.0, v_peri, 0.0);
        let period = std::f64::consts::TAU * (a * a * a / MU_SUN).sqrt();

        let mut model = CompositeForce::new().with(Box::new(PointMassGravity::new(vec![(
            MU_SUN,
            FixedPerturber::at_origin(),
        )
            .into()])));
        if let Some((a_tow, dir)) = tug {
            model = model.with(Box::new(GravityTractor::sun_at_origin(a_tow, dir, always())));
        }

        let stepper = Dop853::new().with_tolerances(1e-13, 1e-6);
        let mut samples = Vec::with_capacity(n_orbits + 1);
        let mut epoch = epoch0();
        samples.push(osculating_a(&state, MU_SUN));
        for _ in 0..n_orbits {
            state = stepper.step(&model, epoch, &state, period).unwrap();
            epoch = epoch.shifted_by_seconds(period);
            samples.push(osculating_a(&state, MU_SUN));
        }
        slope_per_step(&samples) / period
    }

    /// The de-risk case: on a circular orbit a constant transverse tug drives
    /// `da/dt = 2·a_T/n` exactly, with no time-weighting subtlety at all. This
    /// validates the term's form, sign, and units against arithmetic.
    ///
    /// Amplified to `a_tow = 1e-9` (≈40× a real 20 t tractor) purely so the drift
    /// clears integrator noise over a few dozen orbits — the term is linear in
    /// `a_tow`, so this tests the same physics the shipping magnitude uses.
    #[test]
    fn circular_orbit_drifts_at_the_constant_transverse_rate() {
        let a_tow = 1e-9;
        let a = 1.0 * AU;
        let measured = measure_secular_da_dt(Some((a_tow, TowDirection::Prograde)), a, 0.0, 40);
        let oracle = closed_form_constant_circular(a_tow, a, MU_SUN);
        assert!(measured > 0.0, "prograde tug must raise a, got {measured}");
        let rel = (measured - oracle).abs() / oracle;
        assert!(
            rel < 0.01,
            "measured {measured:.6e} m/s vs oracle {oracle:.6e} (rel {rel:.4})"
        );
    }

    /// The eccentric case, judged against the shared **time-averaged** oracle at
    /// `d = 0`. This is where the tractor and Yarkovsky genuinely share machinery:
    /// same Gauss equation, same uniform-in-mean-anomaly weighting, only the
    /// exponent differs.
    #[test]
    fn eccentric_orbit_matches_the_time_averaged_oracle_at_d_zero() {
        let a_tow = 1e-9;
        let (a, e) = (1.0 * AU, 0.2);
        let measured = measure_secular_da_dt(Some((a_tow, TowDirection::Prograde)), a, e, 40);
        let oracle = time_averaged(a_tow, AU, 0.0, a, e, MU_SUN);
        assert!(measured > 0.0, "prograde tug must raise a, got {measured}");
        let rel = (measured - oracle).abs() / oracle;
        assert!(
            rel < 0.01,
            "measured {measured:.6e} m/s vs oracle {oracle:.6e} (rel {rel:.4})"
        );
    }

    /// Sign check on the mission choice: station-keeping behind the asteroid
    /// lowers the semi-major axis.
    #[test]
    fn retrograde_station_keeping_drifts_inward() {
        let measured =
            measure_secular_da_dt(Some((1e-9, TowDirection::Retrograde)), 1.0 * AU, 0.15, 40);
        assert!(measured < 0.0, "retrograde tug must lower a, got {measured}");
    }

    /// The guard that gives the secular tests meaning: the identical integration
    /// with the tractor absent must drift by a small fraction of the signal, else
    /// a loose tolerance would be "measuring" integrator noise.
    #[test]
    fn control_without_the_tractor_shows_no_drift() {
        let (a, e) = (1.0 * AU, 0.2);
        let control = measure_secular_da_dt(None, a, e, 40);
        let signal = measure_secular_da_dt(Some((1e-9, TowDirection::Prograde)), a, e, 40);
        assert!(
            control.abs() < 0.01 * signal.abs(),
            "control drift {control:.3e} m/s must be ≪ signal {signal:.3e} m/s"
        );
    }
}

