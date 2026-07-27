//! `MissionCore` — the godot-free heart of the binding.
//!
//! Holds the loaded DE440 ephemeris and (once built) the [`RealFieldScenario`],
//! and answers the two questions the Godot frontend asks: *where is body N at
//! epoch t* (for the solar-system display) and *how much along-track Δv clears
//! the threat at this lead* (the headline number + planner). It deals only in
//! plain Rust / nalgebra types, so it is unit-testable with `cargo test` — no
//! running Godot. The thin [`crate::Mission`] class marshals these to Godot types
//! and never adds logic of its own.
//!
//! **`RealFieldScenario` is `Send`** — the core traits (`ForceModel`,
//! `PerturberEphemeris`, `GeocentricState`) carry `Send + Sync` bounds, pinned by
//! a compile-time assertion in `core::scenario`. A built scenario can therefore be
//! produced on a worker thread and moved here, which is the only way the ~10 s
//! build does not freeze Godot's main thread. (This note previously said the
//! opposite; the bounds were added once the measurement showed the build was far
//! too slow to run inline.)
//!
//! What must **not** move to a worker is *this* struct: it serves planet positions
//! (`body_position_ecl_au`) every frame from `load()`, which is ~19 ms and live
//! immediately. Sending it away for the duration of a build would freeze the
//! orrery for those 10 s — the very regression threading exists to prevent. The
//! split that follows from it: clone the `Arc<Ephemeris>`, build a scenario
//! off-thread from that clone, and hand the finished scenario back to this
//! (still-serving) core to install.
//!
//! **Two-phase, on purpose.** [`load`](MissionCore::load) reads the kernels
//! (~ms) and immediately enables body-position queries; [`build_scenario`](
//! MissionCore::build_scenario) runs the expensive multi-year back-propagation
//! that the deflection solver needs. Splitting them lets the display show the
//! real planets the instant the kernel is in, while the scenario builds behind
//! it — and lets the fast path be exercised without paying the slow one.
//!
//! **Frame:** the core works in ICRF (equatorial J2000); the display draws in
//! the **ecliptic** plane. [`icrf_km_to_ecliptic_au`] applies the fixed J2000
//! obliquity rotation (SPICE's `ECLIPJ2000` value, 84381.448″) so the returned
//! positions sit in the ecliptic — skipping it would tilt the whole system ~23°.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use anise::constants::frames::{SSB_J2000, SUN_J2000};
use anise::prelude::Frame;
use godot::global::godot_warn;
use nalgebra::Vector3;

use asteroid_core::deflection::DeflectionError;
use asteroid_core::ephemeris::Ephemeris;
use asteroid_core::geometry::BPlaneEncounter;
use asteroid_core::horizons::Neo;
use asteroid_core::launch_vehicle::{LaunchVehicle, LAUNCH_VEHICLES};
use asteroid_core::mission::{
    cell_delivery, porkchop_grid, required_impactor_mass, verify_cell, MassSolveOutcome, Porkchop,
    PorkchopCell, TransferMetrics,
};
use asteroid_core::scenario::{
    DeflectedArc, EncounterFrame, ImpactorConfig, RealFieldScenario, ScenarioError, SrpParams,
    Tier2Config, ENCOUNTER_HALF_WINDOW_SECONDS, ENCOUNTER_SAMPLES, SAFE_PERIGEE_TARGET_M,
};
use asteroid_core::{
    along_track_unit, Clock, DvSolveTol, EphemerisPerturber, Epoch, OrbitalElements, StateVector,
};

/// Kilometres per astronomical unit — the display scale positions cross into.
const AU_KM: f64 = 1.495_978_707e8;
/// Metres per kilometre — the integrated `Clock` stores SSB positions in metres,
/// but [`icrf_km_to_ecliptic_au`] takes kilometres, so we scale down first.
const M_PER_KM: f64 = 1.0e3;
/// Mean obliquity of the ecliptic at J2000, arcseconds — the exact value that
/// defines SPICE's `ECLIPJ2000` frame, so our ecliptic matches the kernel's.
const OBLIQUITY_ARCSEC: f64 = 84_381.448;

/// Rotate an ICRF (equatorial-J2000) position in **km** into ecliptic-J2000 and
/// scale to **AU**. A rotation by the mean obliquity about the shared X axis
/// (vernal equinox): the ecliptic north pole sits at ICRF `(0, −sinε, cosε)`.
pub fn icrf_km_to_ecliptic_au(v_km: Vector3<f64>) -> Vector3<f64> {
    let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
    let (s, c) = eps.sin_cos();
    Vector3::new(
        v_km.x / AU_KM,
        (c * v_km.y + s * v_km.z) / AU_KM,
        (-s * v_km.y + c * v_km.z) / AU_KM,
    )
}

/// Rotate an **ecliptic-J2000** vector into ICRF (equatorial-J2000) — the exact
/// inverse of the rotation in [`icrf_km_to_ecliptic_au`] (a `+ε` about X vs the
/// forward `−ε`), and **unit-agnostic**: it rotates a vector, so it maps both a
/// position and a velocity. The synthetic-body seed path needs it: a designer
/// orbit is authored with its inclination referred to the *ecliptic* (the plane
/// the display and a human designer think in), but the integrator runs in ICRF,
/// so the element→state result is rotated here before it is seeded.
fn ecliptic_to_icrf(v: Vector3<f64>) -> Vector3<f64> {
    let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
    let (s, c) = eps.sin_cos();
    Vector3::new(v.x, c * v.y - s * v.z, s * v.y + c * v.z)
}

/// The ecliptic north pole **expressed in ICRF** — `(0, −sin ε, cos ε)`.
///
/// Not `(0, 0, 1)`. That is the pole in *ecliptic* coordinates; here it is only
/// ever dotted against ICRF vectors, and the two differ by the 23.4° obliquity.
/// This is [`icrf_km_to_ecliptic_au`]'s rotation read backwards: that function maps
/// ICRF `(0, −sin ε, cos ε)` onto ecliptic `(0, 0, 1)`.
fn ecliptic_north_icrf() -> Vector3<f64> {
    let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
    let (s, c) = eps.sin_cos();
    Vector3::new(0.0, -s, c)
}

/// The b-plane display basis `(ξ̂, ζ̂, Ŝ)` — three ICRF unit vectors.
///
/// `Ŝ` is the core's incoming-asymptote direction for the nominal encounter; the
/// b-plane is the plane through Earth's centre perpendicular to it. The two
/// in-plane axes are a **display** choice, not physics. The core deliberately
/// leaves the Öpik/Kizner ξ,ζ decomposition *and the b-vector's sign* unpinned
/// (`geometry.rs` §10.8) because settling them is a keyhole/covariance question
/// this view does not ask. So these are built the conventional way — ξ̂ ∝ Ŝ × N̂
/// against the ecliptic pole, ζ̂ = Ŝ × ξ̂ — and treated as what they are: a frame to
/// draw in. Everything the view *reports* (|B|, perigee, capture radius, v_inf) is
/// a rotation-invariant scalar from the core, so no number a player reads depends
/// on the choice made here; only which way the picture happens to be turned does.
///
/// **Everything in this function is ICRF, and that is load-bearing.** The tracks
/// being projected are geocentric ICRF (the integration frame with Earth's position
/// subtracted) and `Ŝ` is ICRF, so the reference pole must be the ecliptic north
/// pole *in ICRF* ([`ecliptic_north_icrf`]) — never the ecliptic-frame `(0, 0, 1)`,
/// and the tracks must never be run through [`icrf_km_to_ecliptic_au`] on the way
/// in. Mixing the two frames here would tilt the plot by the obliquity: a picture
/// that still looks like a plausible encounter, which is the whole danger.
///
/// Returns `None` only for a non-finite `Ŝ`. A `Ŝ` parallel to the ecliptic pole
/// (an encounter straight down from ecliptic north) leaves ξ̂ undefined by this
/// recipe rather than wrong, so it falls back to another reference axis: the pole
/// is arbitrary for a *display* frame, and no readout depends on it.
fn bplane_basis(s_hat: Vector3<f64>) -> Option<(Vector3<f64>, Vector3<f64>, Vector3<f64>)> {
    if !s_hat.iter().all(|c| c.is_finite()) || s_hat.norm() < 1e-12 {
        return None;
    }
    let s = s_hat.normalize();

    // ξ̂ ∝ Ŝ × N̂. Degenerate only when Ŝ ∥ N̂; then any perpendicular will do, and
    // two candidate axes cannot both be parallel to Ŝ.
    let mut xi = s.cross(&ecliptic_north_icrf());
    if xi.norm() < 1e-9 {
        xi = s.cross(&Vector3::x());
    }
    if xi.norm() < 1e-9 {
        xi = s.cross(&Vector3::y());
    }
    if xi.norm() < 1e-9 {
        return None;
    }
    let xi_hat = xi.normalize();
    // Ŝ ⊥ ξ̂ already, so this is unit without renormalising.
    let zeta_hat = s.cross(&xi_hat);
    Some((xi_hat, zeta_hat, s))
}

/// Project a geocentric **ICRF** vector in metres onto the b-plane display basis,
/// returning `(ξ, ζ, s)` in **kilometres**.
///
/// `s` — the component along the incoming asymptote — is the depth axis: negative
/// inbound, positive outbound, so a consumer can shade the approach and pick out
/// the b-plane crossing without knowing any geometry.
///
/// The f64→f32 boundary is respected the same way the rest of the binding respects
/// it (HANDOFF §7): the subtraction that produced this geocentric vector happened
/// in f64 inside the core, and only the small residual crosses to Godot. At the
/// scale that matters (a ~10⁴ km perigee) f32's ~1e-7 relative precision is
/// millimetres.
fn project_bplane(
    g_m: Vector3<f64>,
    basis: (Vector3<f64>, Vector3<f64>, Vector3<f64>),
) -> Vector3<f64> {
    Vector3::new(g_m.dot(&basis.0), g_m.dot(&basis.1), g_m.dot(&basis.2)) / M_PER_KM
}

/// Discover the loaded kernel's usable coverage window by bisecting on whether
/// Earth resolves — `(lo, hi)` seconds past J2000, inset by [`SPAN_MARGIN_S`].
///
/// Bisection rather than a hardcoded date pair because the mounted kernel decides
/// the answer: de440s covers ~1850–2149, de441 ~1550–2650, and hardcoding the
/// short span would silently cap a user who mounted the long one. Bisection
/// rather than reading the SPK segment headers because coverage is only *useful*
/// where a full geocenter lookup succeeds (SSB→EMB→Earth — all three segments),
/// which is exactly what this probes; a segment table can advertise a span the
/// dereferencing chain cannot actually serve.
///
/// ~40 lookups at ~µs each, once per load. Errors only if the kernel serves no
/// epoch at all (a wrong or corrupt file), which is worth failing loudly on.
fn discover_span(eph: &Ephemeris) -> Result<(f64, f64), ScenarioError> {
    let resolves = |t: f64| -> bool {
        eph.position_km(
            Frame::from_ephem_j2000(399),
            SUN_J2000,
            Epoch::from_tdb_seconds_past_j2000(t).as_hifitime(),
        )
        .is_ok()
    };

    // A kernel that serves nothing anywhere is a load failure, not an empty span.
    if !resolves(0.0) {
        return Err(ScenarioError::Ephemeris(
            "kernel resolves no Earth position at J2000 — wrong or corrupt file?".into(),
        ));
    }

    // Walk each edge in from a bracket known to be outside coverage. J2000 is
    // inside (checked above), so each bisection is well-posed.
    let mut lo = (PROBE_LO_S, 0.0); // (fails, works)
    while lo.1 - lo.0 > SPAN_MARGIN_S {
        let mid = 0.5 * (lo.0 + lo.1);
        if resolves(mid) {
            lo.1 = mid
        } else {
            lo.0 = mid
        }
    }
    let mut hi = (0.0, PROBE_HI_S); // (works, fails)
    while hi.1 - hi.0 > SPAN_MARGIN_S {
        let mid = 0.5 * (hi.0 + hi.1);
        if resolves(mid) {
            hi.0 = mid
        } else {
            hi.1 = mid
        }
    }
    Ok((lo.1 + SPAN_MARGIN_S, hi.0 - SPAN_MARGIN_S))
}

/// Where a catalog body's positions come from — and, because the two answer in
/// **different frames**, a distinction the read path is not allowed to forget.
///
/// This is the one place the project mixes provenance in a single list, so each
/// variant states what it is and what it is not:
///
/// - [`Integrated`](Trajectory::Integrated) — *our* physics. A synthetic designer
///   body flown through the same validated Tier-1 field as the threat. States are
///   **SSB metres**, the integration frame.
/// - [`Sampled`](Trajectory::Sampled) — *JPL's* physics. A real asteroid read from
///   a Horizons state table and interpolated between JPL's own samples; nothing
///   here is integrated by us. States are **heliocentric ICRF metres**.
///
/// The frames differ by the Sun's barycentric wobble (~10⁶ km) — large enough to
/// misplace a body, small enough to look like a rendering nudge rather than a bug.
/// `catalog_body_helio_ecl_au` is the single conversion point, so there is exactly
/// one place that has to get it right.
///
/// A frontend that wants to *say* which is which reads
/// [`MissionCore::catalog_provenance`]: this project's standing rule is that
/// nothing is drawn beside real physics without being labelled, which is what the
/// deleted GDScript Kepler violated.
pub enum Trajectory {
    /// Integrated by us, in our field. SSB metres.
    Integrated(Clock),
    /// JPL Horizons states, interpolated. Heliocentric ICRF metres.
    Sampled(Neo),
}

/// One body in the orrery catalog: a trajectory the display scrubs over, built
/// **once** and thereafter only evaluated — a scrub query is a dense-output
/// evaluation or a Hermite interpolation, never a re-integration and never a
/// re-read of the source file.
pub struct OrreryBody {
    /// Display label (e.g. `"C/2029 K1"`, `"99942 Apophis"`).
    name: String,
    /// Coarse class the frontend styles on (`"asteroid"`, `"comet"`, …).
    kind: String,
    /// Where the positions come from — and in which frame. See [`Trajectory`].
    trajectory: Trajectory,
}

/// The display comet **C/2029 K1** — the one piece of scenery in the orrery, and
/// the parameters it is authored from.
///
/// It exists to give the long clock something to sweep besides the planets, and it
/// is *synthetic and labelled as such*: a designed orbit flown through the same
/// validated Tier-1 field as the threat, not a real object. What it is emphatically
/// not any more is a Kepler ellipse drawn in GDScript beside a real integrated
/// threat — two different physics on one screen with nothing marking which is which.
pub mod display_comet {
    use super::*;
    use crate::AU_M;

    pub const NAME: &str = "C/2029 K1";
    pub const KIND: &str = "comet";

    const A_AU: f64 = 8.0; // ⇒ period ≈ 22.6 yr
    const E: f64 = 0.9; // q ≈ 0.8 AU, Q ≈ 15.2 AU
    const INCLINATION_DEG: f64 = 28.0;
    const RAAN_DEG: f64 = 210.0;
    const ARG_PERIAPSIS_DEG: f64 = 0.0;

    /// True anomaly at the campaign-start epoch, degrees — just past aphelion and
    /// inbound, chosen so **perihelion falls near the impact epoch** rather than in
    /// the campaign's first months. The display's job is to have something happening
    /// while the operator works, and the operator works towards impact.
    ///
    /// Derived from that intent, not tuned by eye: wanting perihelion ≈ 12.8 yr
    /// after epoch0 on a 22.6 yr period fixes the mean anomaly at
    /// `M₀ = −2π·(12.8/22.6) ≈ 2.725 rad`, and solving Kepler's equation at `e = 0.9`
    /// gives `E ≈ 2.921 rad` ⇒ `ν ≈ 176.8°`. Elements here are true-anomaly only by
    /// design (`elements.rs`), which is why the conversion is spelled out rather
    /// than computed.
    ///
    /// **Measured on the real perturbed field** (not the two-body derivation):
    /// perihelion **0.807 AU, +0.97 yr after impact** — the comet is inbound at
    /// ~4.4 AU while the encounter plays out and rounds the Sun the year after.
    /// `build_worker_installs_the_display_comet` re-measures it.
    const TRUE_ANOMALY_DEG: f64 = 176.8;

    /// Snapshot cadence, seconds. Sub-cadence scrub queries come from the
    /// integrator's dense output, so this sets memory and integration cost — not the
    /// fidelity of the fast perihelion rounding.
    pub const CADENCE_SECONDS: f64 = 5.0 * 86_400.0;

    /// Snapshots — **one full orbit** (≈ 22.6 yr) from the campaign start. Measured
    /// cost: ~4 s of integration, which is why this rides the build worker and never
    /// the main thread. One period rather than two because a second lap retraces the
    /// same arc for another ~4 s of build.
    pub const N_SNAPSHOTS: u32 = 1651;

    /// The authored orbit, referred to the ecliptic at the campaign-start epoch.
    pub fn elements() -> OrbitalElements {
        OrbitalElements::new(
            A_AU * AU_M,
            E,
            INCLINATION_DEG.to_radians(),
            RAAN_DEG.to_radians(),
            ARG_PERIAPSIS_DEG.to_radians(),
            TRUE_ANOMALY_DEG.to_radians(),
        )
    }
}

/// Integrate a synthetic designer body into an [`OrreryBody`], given the field it
/// flies in — the seed math, factored out so the **worker thread** and
/// [`MissionCore::add_synthetic_body`] cannot drift apart.
///
/// The seed is built in the integration frame: element→state about the Sun
/// (heliocentric, ecliptic), rotate ecliptic→ICRF, add the Sun's SSB state — the
/// exact inverse of the read path, so a query back at `epoch0` recovers the
/// authored position. See [`MissionCore::add_synthetic_body`] for the parameters.
///
/// Free rather than a method because the worker holds a `BuiltScenario` and an
/// `Arc<Ephemeris>` but **no `MissionCore`** — the core stays on the main thread
/// serving the orrery while this runs.
pub fn seed_orrery_body(
    ephemeris: &Arc<Ephemeris>,
    scenario: &RealFieldScenario,
    name: &str,
    kind: &str,
    elements: OrbitalElements,
    epoch0: Epoch,
    cadence_seconds: f64,
    n_snapshots: u32,
) -> Result<OrreryBody, ScenarioError> {
    let mu_sun = ephemeris
        .sun_gm_m3_s2()
        .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;

    let helio_ecl = elements.to_state(mu_sun);
    let helio_icrf = StateVector::new(
        ecliptic_to_icrf(helio_ecl.position),
        ecliptic_to_icrf(helio_ecl.velocity),
    );
    let sun_ssb = EphemerisPerturber::new(Arc::clone(ephemeris), SUN_J2000)
        .state_at(epoch0)
        .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
    let seed = StateVector::new(
        helio_icrf.position + sun_ssb.position,
        helio_icrf.velocity + sun_ssb.velocity,
    );

    Ok(OrreryBody {
        name: name.to_string(),
        kind: kind.to_string(),
        trajectory: Trajectory::Integrated(scenario.propagate_free(
            epoch0,
            seed,
            cadence_seconds,
            n_snapshots,
        )?),
    })
}

/// Load every real near-Earth asteroid state table on disk into catalog bodies.
///
/// **No scenario, no almanac, no integration** — a `.neo` table already contains
/// JPL's trajectory, so this is a file read and nothing more. That is the whole
/// difference between these and [`seed_orrery_body`]'s synthetic scenery, and it
/// is why real asteroids cost no build time and cannot perturb the threat.
///
/// A table that fails to parse costs that one asteroid and is warned about; it
/// never takes the build down, for the same reason a failed small-body mount does
/// not. Returns an empty vector when no tables are present, which is the ordinary
/// state of a fresh clone (`python pyref/fetch_horizons_neo.py` fetches them).
///
/// Runs on the build worker only because that is where the catalog is assembled —
/// it is milliseconds, not the small-body kernel's seconds.
pub fn load_neo_bodies() -> Vec<OrreryBody> {
    let (bodies, errors) = asteroid_core::horizons::load_all();
    for e in errors {
        godot_warn!("NEO state table skipped: {e}");
    }
    bodies
        .into_iter()
        .map(|neo| OrreryBody {
            name: neo.name().to_string(),
            kind: "asteroid".to_string(),
            trajectory: Trajectory::Sampled(neo),
        })
        .collect()
}

/// A committed deflection plan and its precomputed result.
///
/// `encounter == None` is the **clean-miss success case** — the deflected pass left
/// the scan gate, i.e. the miss is so wide it is off any sensible frame, which is
/// exactly what the player wants. It must stay distinct from "no plan set" (that
/// is `MissionCore::plan == None`), so the planner does not read the *best*
/// deflection as a failure.
struct PlanState {
    /// The deflection epoch, seconds past J2000 — before this the impulse has not
    /// happened, so deflected queries fall back to the nominal track.
    deflection_seconds: f64,
    /// The post-impulse arc: a `Clock` covering `[deflection_epoch, span_end]`.
    clock: Clock,
    /// The b-plane geometry of the deflected pass — impact parameter, perigee,
    /// `v_inf`, the focused capture disc, and the core's own `is_hit()`. `None` for
    /// a clean miss that left the scan gate (see the struct note).
    encounter: Option<BPlaneEncounter>,
    /// Both tracks in Earth's frame over the encounter window. Built from the
    /// **same** [`DeflectedArc`] as `encounter`, so the pass the b-plane view draws
    /// and the numbers annotating it cannot describe different propagations — the
    /// invariant `frame_from` used to hold internally, kept here now that the
    /// propagation happens in `set_plan`.
    frame: EncounterFrame,
}

/// A safety margin pulled in from each discovered coverage edge, seconds (1 day).
/// The bisection lands within a day of the true edge; insetting by that much
/// guarantees the reported span is *inside* coverage rather than straddling it,
/// so a clock clamped to this span never asks for an epoch the kernel lacks.
const SPAN_MARGIN_S: f64 = 86_400.0;

/// Bisection bounds for span discovery, seconds past J2000 — years ~1000 and
/// ~3000, comfortably outside any DE kernel's coverage (de440s ≈ 1850–2149;
/// de441 ≈ 1550–2650), so the true edges are always bracketed.
const PROBE_LO_S: f64 = -31_557_600_000.0;
const PROBE_HI_S: f64 = 31_557_600_000.0;

/// The bodies `sb441-n16.bsp` actually contains: the 16 asteroid perturbers ASSIST
/// integrates against, by NAIF id. **Main-belt, every one of them** — this file is
/// a perturber set, not a target list, so there is no Apophis and no Bennu here and
/// there never will be. Those are per-object JPL Horizons kernels, read through this
/// same path once they are fetched.
///
/// Ids are the SPK convention for a numbered asteroid: 2000000 + the number, so
/// 2000001 is (1) Ceres. Verified by enumerating the kernel's segment table rather
/// than copied from documentation.
pub const SB441_BODIES: &[(i32, &str)] = &[
    (2000001, "Ceres"),
    (2000002, "Pallas"),
    (2000003, "Juno"),
    (2000004, "Vesta"),
    (2000007, "Iris"),
    (2000010, "Hygiea"),
    (2000015, "Eunomia"),
    (2000016, "Psyche"),
    (2000031, "Euphrosyne"),
    (2000052, "Europa"),
    (2000065, "Cybele"),
    (2000087, "Sylvia"),
    (2000088, "Thisbe"),
    (2000107, "Camilla"),
    (2000511, "Davida"),
    (2000704, "Interamnia"),
];

/// Build an almanac with the small-body kernel chained onto the DE pair — the
/// worker-thread counterpart to [`MissionCore::load_from`].
///
/// **~5.7 s cold, ~272 ms warm.** The gap is page-cache I/O on a 646 MB file, so a
/// freshly launched game pays the full cost and only a re-run pays the small one.
/// That measurement is why this exists as a separate function called from the build
/// worker instead of a line inside `load_from`.
///
/// Re-reading de440s + pck11 (~ms) rather than mounting onto the served `Arc` is not
/// waste — [`Ephemeris::with_constants`] takes `self` by value, and the served
/// almanac is behind an `Arc` that the render thread is reading every frame. A
/// second almanac is the only way to do this without stopping the solar system.
pub fn mount_small_bodies(
    bsp: &Path,
    pca: &Path,
    small_bodies: &Path,
) -> Result<Ephemeris, ScenarioError> {
    let to_str = |p: &Path| -> Result<String, ScenarioError> {
        p.to_str()
            .map(str::to_owned)
            .ok_or_else(|| ScenarioError::Ephemeris(format!("kernel path is not UTF-8: {p:?}")))
    };
    Ephemeris::load(&to_str(bsp)?)
        .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
        .with_constants(&to_str(pca)?)
        .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
        .with_constants(&to_str(small_bodies)?)
        .map_err(|e| ScenarioError::Ephemeris(e.to_string()))
}

/// Every term id [`MissionCore::tier2_shifted_perigee_m`] answers to, in panel
/// order — the frontend's `TIER2_TERMS` table names the same five.
///
/// Exists so a new term cannot be measured, wired and displayed while the tests
/// quietly keep checking the old set: the preview test loops over *this*, so adding
/// a field to [`Tier2Shifts`] without adding it here leaves a visibly short list
/// rather than a green run that never touched the new physics. That is not
/// hypothetical — the `J2` term was added with the loop still reading
/// `["relativity", "yarkovsky", "srp"]`, and it passed.
pub const TIER2_TERM_IDS: [&str; 5] = ["relativity", "yarkovsky", "belt", "srp", "j2"];

/// The shipping Yarkovsky `A2` the Tier-2 preview toggles on (m/s² at 1 AU) — a
/// physically plausible sub-km value, deliberately **un-amplified** (matches the
/// core `tier2_terms_…_shift_it_on` measurement test). A larger value would inflate
/// the displayed shift into the display-grade lie this project keeps catching.
pub const PREVIEW_YARKOVSKY_A2: f64 = 1.0e-13;

/// The five single-term Tier-2 b-plane perigees, measured on demand when the
/// frontend opens the force-model menu ([`measure_tier2_shifts`]).
///
/// Each is the nominal perigee the **fixed shipping seed** reaches when *one*
/// Tier-2 term is switched on and the rest left off — the honest measurement of how
/// far that single piece of physics moves the predicted impact (HANDOFF §5/§6),
/// computed via [`RealFieldScenario::nominal_encounter_with`] (seed fixed, only the
/// forward field changed; never a rebuild, which would re-design the seed through
/// the term and hide the shift). The frontend shows the shift as
/// `nominal_perigee − this`; the sign says which way (inward = smaller perigee =
/// pulled closer to Earth).
///
/// Each measurement is a full ~16 s propagation, so all five together are the ~80 s
/// this preview costs — paid **once**, off-thread, only when the flag is set.
#[derive(Debug, Clone, Copy)]
pub struct Tier2Shifts {
    /// Nominal perigee with 1PN relativity alone on, m.
    pub relativity_perigee_m: f64,
    /// Nominal perigee with Yarkovsky ([`PREVIEW_YARKOVSKY_A2`]) alone on, m.
    pub yarkovsky_perigee_m: f64,
    /// Nominal perigee with the sb441 belt alone on, m — **`None` when the
    /// small-body kernel was not mounted**. The belt shift is then genuinely
    /// *unavailable*, not zero: reporting 0 would read as "the belt does nothing,"
    /// the same display-grade lie in miniature.
    pub belt_perigee_m: Option<f64>,
    /// Nominal perigee with SRP ([`SrpParams::sub_km_rock`]) alone on, m.
    pub srp_perigee_m: f64,
    /// Nominal perigee with Earth's `J2` oblateness alone on, m.
    ///
    /// Measured on the *same* fixed shipping seed as its four siblings, because the
    /// frontend subtracts every one of them from the *same* baseline — a term
    /// measured on some other geometry would difference two unrelated numbers and
    /// print something that looks like a shift and is not.
    ///
    /// That is also what makes this entry the one with a caveat: the shipping
    /// nominal is a designed **impact** whose closest approach is 3000 km, inside
    /// Earth, and the `J2` expansion is only valid *outside* `R_eq`. The number is
    /// real and it is what this geometry does; the in-domain companion is measured
    /// on a deflected miss by the core
    /// ([`J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM`](asteroid_core::scenario::J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM)),
    /// which is what the panel's footnote cites.
    pub j2_perigee_m: f64,
}

/// A finished scenario and everything fixed at build time, ready to be handed to a
/// [`MissionCore`] — the unit of work that crosses back from a worker thread.
///
/// Exists because the ~10 s build must not run on the render thread, and neither
/// must it take the `MissionCore` with it: that core is serving planet positions
/// every frame, so it stays put and receives this when it lands (see the module
/// note). `RealFieldScenario` is `Send` (pinned by an assertion in the core) and
/// `BPlaneEncounter` is `Copy`, so this whole struct moves between threads freely.
///
/// Everything expensive and *invariant* is computed here, on the worker, once:
/// the back-propagated seed, the nominal trajectory, and the nominal encounter
/// scan. None of the three can change for a given scenario, so no display read and
/// no planner nudge should ever recompute one.
pub struct BuiltScenario {
    scenario: RealFieldScenario,
    /// The nominal trajectory, cloned out of the scenario's own (now warm) cache.
    nominal_clock: Clock,
    /// The nominal encounter — the hit being undone. Scanned here rather than on
    /// demand: it is what [`MissionCore::capture_radius_m`] reports, and a full-span
    /// scan is not something a planner readout can pay for.
    nominal_encounter: BPlaneEncounter,
    /// The pre-plan encounter picture: the nominal track in Earth's frame, with no
    /// deflection anywhere (`frame.deflected` is empty). Sampled here because it is
    /// as invariant as the nominal itself — the incoming impact never changes — so
    /// the b-plane view can open on the threat the instant the build lands, with no
    /// propagation and nothing to wait for.
    nominal_frame: EncounterFrame,
    /// The almanac this was built against, carried back so [`MissionCore::install`]
    /// can adopt it. When the worker mounted the small-body kernel this is the
    /// *mounted* almanac, and adopting it is what puts asteroids within reach of
    /// `body_position_ecl_au`; when it did not, this is the same `Arc` that went
    /// out and the swap is a no-op refcount bump.
    ///
    /// It travels **with** the scenario for the same reason the orrery bodies do:
    /// the scenario was integrated in this field, so a core serving positions from
    /// a different one would be quietly answering from a field its own threat was
    /// never flown in.
    ephemeris: Arc<Ephemeris>,
    /// Whether [`ephemeris`](Self::ephemeris) has the small-body kernel on it.
    small_bodies_mounted: bool,
}

impl BuiltScenario {
    /// The field just built, borrowed — so the **same worker** can fly orrery bodies
    /// through it before handing everything over ([`seed_orrery_body`]). Borrowing
    /// rather than moving keeps the scenario whole for [`MissionCore::install`].
    pub fn scenario_ref(&self) -> &RealFieldScenario {
        &self.scenario
    }

    /// The campaign-start epoch this scenario was built around — the epoch orrery
    /// bodies are seeded at, so the scenery and the campaign share one `t = 0`.
    pub fn epoch0(&self) -> Epoch {
        self.scenario.epoch0()
    }

    /// Design the impactor, back-propagate the seed, fly the nominal, and scan the
    /// encounter it produces — **~10 s of work**, and the whole reason this takes an
    /// `Arc<Ephemeris>` rather than `&MissionCore`: it is meant to be called on a
    /// worker thread, from a clone of the almanac, while the core it will eventually
    /// feed keeps drawing the solar system.
    ///
    /// The `small_bodies_mounted` flag describes the almanac being handed in; it is
    /// carried, not inferred. Probing the almanac for an asteroid to find out would
    /// be a lookup that fails for two different reasons (not mounted, or out of
    /// span) and reports one.
    pub fn build(
        eph: Arc<Ephemeris>,
        cfg: &ImpactorConfig,
        small_bodies_mounted: bool,
    ) -> Result<Self, ScenarioError> {
        let ephemeris = Arc::clone(&eph);
        let scenario = RealFieldScenario::build_with(cfg, eph)?;
        // `build_with` already verified its round-trip through `deflection()`, so the
        // scenario's nominal cache is warm and all of these are cheap reads of work
        // already done — not a third propagation. The frame adds only ~1400 dense
        // evaluations and ephemeris look-ups (milliseconds against a ~10 s build).
        let (nominal_clock, nominal_encounter, nominal_frame) = {
            let ds = scenario.deflection()?;
            let enc = scenario.nominal_hit(&ds)?;
            let frame = scenario.frame_from_arcs(
                ds.nominal(),
                enc,
                None, // no plan exists at build time — the pre-plan picture
                ENCOUNTER_HALF_WINDOW_SECONDS,
                ENCOUNTER_SAMPLES,
            )?;
            (ds.nominal().clone(), enc, frame)
        };
        Ok(Self {
            scenario,
            nominal_clock,
            nominal_encounter,
            nominal_frame,
            ephemeris,
            small_bodies_mounted,
        })
    }
}

/// Measure the five single-term Tier-2 b-plane shifts off a built scenario
/// (HANDOFF §5). Each term is re-flown in isolation on the **fixed shipping seed**
/// via [`RealFieldScenario::nominal_encounter_with`] — the honest "how far does this
/// piece of physics move the impact" measurement, never a rebuild that would
/// reproduce the hit by construction.
///
/// `&RealFieldScenario` rather than `&BuiltScenario` so the frontend can call it on
/// a **worker thread holding an `Arc` clone** of the exact scenario the threat was
/// flown in (the render thread keeps reading the same scenario meanwhile — the
/// reason it is `Sync`). ~16 s per term, ~80 s total.
///
/// The belt is measured only when `small_bodies_mounted`: `compose_force` fails
/// loud if asked for the sb441 perturbers without the kernel, so its perigee is
/// left `None` (genuinely unavailable, **not** zero — see
/// [`Tier2Shifts::belt_perigee_m`]) otherwise. Every other term is a hard
/// measurement: a perturbed nominal that finds no close approach is a real surprise
/// for a shift this small, so it errors rather than returning a sentinel.
pub fn measure_tier2_shifts(
    scenario: &RealFieldScenario,
    small_bodies_mounted: bool,
) -> Result<Tier2Shifts, ScenarioError> {
    let measure = |cfg: &Tier2Config| -> Result<f64, ScenarioError> {
        scenario
            .nominal_encounter_with(cfg)?
            .map(|e| e.perigee)
            .ok_or_else(|| {
                ScenarioError::Integration(
                    "Tier-2 preview: a perturbed nominal found no close approach".into(),
                )
            })
    };

    let relativity_perigee_m = measure(&Tier2Config {
        relativity: true,
        ..Tier2Config::default()
    })?;
    let yarkovsky_perigee_m = measure(&Tier2Config {
        yarkovsky_a2: Some(PREVIEW_YARKOVSKY_A2),
        ..Tier2Config::default()
    })?;
    let srp_perigee_m = measure(&Tier2Config {
        srp: Some(SrpParams::sub_km_rock()),
        ..Tier2Config::default()
    })?;
    let j2_perigee_m = measure(&Tier2Config {
        earth_j2: true,
        ..Tier2Config::default()
    })?;
    let belt_perigee_m = if small_bodies_mounted {
        Some(measure(&Tier2Config {
            asteroid_perturbers: true,
            ..Tier2Config::default()
        })?)
    } else {
        None
    };

    Ok(Tier2Shifts {
        relativity_perigee_m,
        yarkovsky_perigee_m,
        belt_perigee_m,
        srp_perigee_m,
        j2_perigee_m,
    })
}

/// The loaded mission: always an ephemeris, optionally a built scenario, and —
/// once built — the cached nominal trajectory and (optionally) a deflection plan.
pub struct MissionCore {
    ephemeris: Arc<Ephemeris>,
    /// The paths this core was loaded from, kept so a worker can build a *second*
    /// almanac with the small-body kernel chained on. It cannot mount onto
    /// `ephemeris`: [`Ephemeris::with_constants`] consumes `self`, and this one is
    /// behind an `Arc` being read every frame. Re-reading de440s costs ~ms.
    bsp: PathBuf,
    pca: PathBuf,
    /// The small-body kernel to mount at build time, if the frontend supplied one
    /// ([`set_small_body_kernel`](Self::set_small_body_kernel)). `None` — the
    /// default — is a complete, working mission with no asteroids in the catalog.
    ///
    /// Deliberately *not* mounted at load: measured **5.7 s cold** on the 646 MB
    /// `sb441-n16.bsp`, and `load_from` is contractually fast because the frontend
    /// calls it on the way to the first drawn frame. The build worker already
    /// spends ~10 s off-thread; this belongs there.
    small_bodies: Option<PathBuf>,
    /// Whether [`ephemeris`](Self::ephemeris) — the almanac being *served* — has
    /// the small-body kernel chained on. Distinct from `small_bodies.is_some()`,
    /// which only says one was *armed*: between `set_small_body_kernel` and the
    /// build landing, a path is known and no asteroid is loadable.
    small_bodies_mounted: bool,
    /// The kernel's usable coverage window, `(lo, hi)` seconds past J2000,
    /// discovered by bisection at load (see [`discover_span`]) rather than
    /// hardcoded — the shipped kernel may be de440s (~1850–2149) or the long-span
    /// de441 (~1550–2650), and the frontend clamps its clock to whatever is
    /// actually mounted.
    span: (f64, f64),
    /// The built scenario, behind an `Arc` so the Tier-2 preview worker can hold a
    /// clone and measure shifts off the *exact* scenario the threat was flown in,
    /// while the render thread keeps reading it every frame. `RealFieldScenario` is
    /// `Sync` (pinned by a core gate test) and every post-build method takes `&self`
    /// with `OnceLock` interior mutability, so the shared read needs no lock.
    scenario: Option<Arc<RealFieldScenario>>,
    /// The nominal (un-deflected) trajectory, cloned **once** at build time so
    /// per-frame position/track reads are cheap `Clock` queries. Rebuilding a
    /// `DeflectionScenario` re-propagates the whole multi-year nominal
    /// (`deflection.rs`), so we never do that on a display read.
    nominal_clock: Option<Clock>,
    /// The nominal encounter, scanned once at build time (see [`BuiltScenario`]).
    /// Fixed for the scenario's life — it is the hit the whole mission exists to
    /// undo — and the source of the capture radius every verdict is measured
    /// against, as well as the `Ŝ` the b-plane display frame is built on.
    nominal_encounter: Option<BPlaneEncounter>,
    /// The pre-plan encounter picture, sampled once at build time (see
    /// [`BuiltScenario`]). The nominal track never changes, so neither does this.
    nominal_frame: Option<EncounterFrame>,
    /// The five single-term Tier-2 shifts, present only after the on-demand preview
    /// has landed ([`adopt_tier2_shifts`](Self::adopt_tier2_shifts)). `None` means
    /// the frontend never opened the menu (or a new scenario was just installed);
    /// belt-within-`Some` may still be `None` when the small-body kernel is absent
    /// (see [`Tier2Shifts`]).
    tier2_shifts: Option<Tier2Shifts>,
    /// The current deflection plan, recomputed only on [`set_plan`](Self::set_plan)
    /// and read cheaply thereafter.
    plan: Option<PlanState>,
    /// The orrery catalog: extra bodies (synthetic designer comets/asteroids now,
    /// real cataloged bodies later) each pre-integrated into a dense-output `Clock`
    /// at add time, so the multi-body display scrubs cheaply. Independent of the
    /// threat/plan; indexed by insertion order.
    bodies: Vec<OrreryBody>,
}

impl MissionCore {
    /// Resolve the DE440 kernels through
    /// [`asteroid_core::kernels::resolve`] — `ASTEROID_DE_KERNEL` +
    /// `ASTEROID_PLANETARY_CONSTANTS` if exported, else a conventional directory
    /// — and hold them. Matches the core tests and the `curve`/viewer binaries,
    /// all of which run from a developer shell.
    ///
    /// **A launched Godot game generally has neither variable set** (they are not
    /// persisted at user or machine level), so the frontend resolves paths itself
    /// and calls [`load_from`](Self::load_from). This stays as the shell/test
    /// entry point.
    pub fn load() -> Result<Self, ScenarioError> {
        let k = asteroid_core::kernels::resolve().ok_or_else(|| {
            ScenarioError::KernelsNotFound(asteroid_core::kernels::not_found_message())
        })?;
        let (bsp, pca) = k.as_strs();
        let mut core = Self::load_from(bsp, pca)?;
        // Arm it if the resolver found one beside the pair — still not mounted, so
        // this stays the fast path. A shell run gets asteroids for free; a shell run
        // on a machine without the 646 MB file is unaffected.
        if let Some(sb) = k.small_bodies_str() {
            core.set_small_body_kernel(sb)?;
        }
        Ok(core)
    }

    /// Read the DE kernels at two explicit paths — the entry point for any caller
    /// that resolves paths itself rather than through the environment (the Godot
    /// frontend, which cannot rely on env vars reaching a double-clicked game).
    ///
    /// Fast (~ms plus a short span bisection): enables
    /// [`body_position_ecl_au`](Self::body_position_ecl_au) immediately; the
    /// scenario is built separately.
    pub fn load_from(bsp: &str, pca: &str) -> Result<Self, ScenarioError> {
        let eph = Ephemeris::load(bsp)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?
            .with_constants(pca)
            .map_err(|e| ScenarioError::Ephemeris(e.to_string()))?;
        let span = discover_span(&eph)?;
        Ok(Self {
            ephemeris: Arc::new(eph),
            bsp: PathBuf::from(bsp),
            pca: PathBuf::from(pca),
            small_bodies: None,
            small_bodies_mounted: false,
            span,
            scenario: None,
            nominal_clock: None,
            nominal_encounter: None,
            nominal_frame: None,
            tier2_shifts: None,
            plan: None,
            bodies: Vec::new(),
        })
    }

    /// The loaded kernel's usable coverage window, `(lo, hi)` seconds past J2000.
    /// The frontend clamps its clock to this — outside it every body lookup fails,
    /// and a failed lookup is indistinguishable from "at the Sun" downstream.
    pub fn usable_span_tdb(&self) -> (f64, f64) {
        self.span
    }

    /// Arm the small-body kernel at `path`, to be mounted by the **next** build
    /// (see [`mount_small_bodies`]). Returns an error if the path is not a file —
    /// a typo that silently produced an empty asteroid catalog would look exactly
    /// like "this kernel has no asteroids in it", which is the wrong lesson.
    ///
    /// Nothing is read here; this is a string assignment. The 5.7 s is paid on the
    /// worker, which is the entire reason the mount is not part of `load_from`.
    pub fn set_small_body_kernel(&mut self, path: &str) -> Result<(), ScenarioError> {
        let p = PathBuf::from(path);
        if !p.is_file() {
            return Err(ScenarioError::KernelsNotFound(format!(
                "small-body kernel not found: {path}"
            )));
        }
        self.small_bodies = Some(p);
        Ok(())
    }

    /// The kernel paths a worker needs to build its own mounted almanac:
    /// `(bsp, pca, small_bodies)`. Owned, so they cross the thread boundary
    /// without borrowing this core — which stays here, drawing planets.
    pub fn kernel_paths(&self) -> (PathBuf, PathBuf, Option<PathBuf>) {
        (
            self.bsp.clone(),
            self.pca.clone(),
            self.small_bodies.clone(),
        )
    }

    /// Whether the almanac currently being served has the small-body kernel on it.
    /// **The catalog gate**: false means asteroid lookups will fail, and a failed
    /// lookup is indistinguishable from "at the Sun" once it reaches the display.
    pub fn small_bodies_mounted(&self) -> bool {
        self.small_bodies_mounted
    }

    /// The loaded almanac, shared. Cloning the `Arc` is how a worker thread gets a
    /// field to build against **without taking this core with it** — the core stays
    /// on the main thread answering `body_position_ecl_au` for the orrery while the
    /// build runs. See [`BuiltScenario::build`].
    pub fn ephemeris_arc(&self) -> Arc<Ephemeris> {
        Arc::clone(&self.ephemeris)
    }

    /// Adopt a scenario built elsewhere (a worker thread; see [`BuiltScenario`]),
    /// along with any orrery bodies flown in that same field on the same worker.
    /// Cheap — every expensive thing already happened off-thread.
    ///
    /// `bodies` arrives **with** the scenario rather than being added afterwards
    /// because a new scenario invalidates the catalog (the old bodies were flown in
    /// the old field), and because seeding them here instead would put a multi-second
    /// integration back on the main thread — the very stall the worker exists to
    /// avoid. See [`seed_orrery_body`].
    pub fn install(&mut self, built: BuiltScenario, bodies: Vec<OrreryBody>) {
        // Adopt the field the scenario was flown in — the swap that puts the
        // small-body kernel under `body_position_ecl_au`. The old `Arc` drops when
        // the last frame that read it is done with it; nothing is mutated underneath
        // the renderer, which is why this can happen mid-flight at all.
        self.ephemeris = built.ephemeris;
        self.small_bodies_mounted = built.small_bodies_mounted;
        self.nominal_clock = Some(built.nominal_clock);
        self.nominal_encounter = Some(built.nominal_encounter);
        self.nominal_frame = Some(built.nominal_frame);
        // A new scenario invalidates any prior Tier-2 preview: the shifts were
        // measured against the old field. They are re-measured on demand when the
        // menu is next opened ([`measure_tier2_shifts`]).
        self.tier2_shifts = None;
        self.scenario = Some(Arc::new(built.scenario));
        self.plan = None; // a new scenario invalidates any prior plan
        self.bodies = bodies; // …and the catalog is replaced, not appended to
    }

    /// Append catalog bodies to the **current** scenario, without replacing it.
    ///
    /// Only legitimate for bodies that carry their own trajectory — i.e.
    /// [`Trajectory::Sampled`] ones, which were never flown in any field of ours
    /// and so cannot be invalidated by which scenario is installed. An
    /// *integrated* body must arrive through [`install`](Self::install) with the
    /// scenario it was flown in; adding one here would put it in a catalog beside
    /// a threat that moves through a different field.
    ///
    /// Cheap by construction: no integration, no almanac access.
    pub fn adopt_bodies(&mut self, bodies: Vec<OrreryBody>) {
        debug_assert!(
            bodies
                .iter()
                .all(|b| matches!(b.trajectory, Trajectory::Sampled(_))),
            "only sampled bodies may be adopted after a scenario is installed"
        );
        self.bodies.extend(bodies);
    }

    /// Build the designer impactor + campaign over the already-loaded ephemeris and
    /// install it — the **blocking** form, ~10 s. Fine for tests and shell tools;
    /// a frontend builds through [`BuiltScenario::build`] on a worker and
    /// [`install`](Self::install)s the result, or it freezes for those 10 s.
    ///
    /// `dead_code`-allowed because the tests below are its only caller: the `Mission`
    /// class exposes no blocking build, precisely so a 10 s main-thread stall cannot
    /// be reached from GDScript. Kept because "build and install, synchronously" is
    /// the natural shape for a test or a shell tool, and writing the two steps out
    /// by hand at each call site would be worse.
    #[allow(dead_code)]
    pub fn build_scenario(&mut self, cfg: &ImpactorConfig) -> Result<(), ScenarioError> {
        // Blocking already, so the small-body mount can be honest here: if one was
        // armed, this build produces a mounted almanac exactly as the worker does.
        // A shell tool that armed a kernel and then found no asteroids would have
        // been told nothing about why.
        let (eph, mounted) = match self.small_bodies.clone() {
            Some(sb) => (
                Arc::new(mount_small_bodies(&self.bsp, &self.pca, &sb)?),
                true,
            ),
            None => (self.ephemeris_arc(), false),
        };
        let built = BuiltScenario::build(eph, cfg, mounted)?;
        self.install(built, Vec::new());
        Ok(())
    }

    /// The nominal encounter's gravitationally-focused capture radius, m — the
    /// radius of Earth's effective collision disc in the b-plane. `None` before a
    /// scenario is installed.
    ///
    /// This is the number a deflection verdict must be measured against: a plan is
    /// safe when the deflected perigee clears *this*, not when it clears Earth's
    /// solid radius (focusing bends a track that would geometrically miss onto the
    /// surface), and not merely when
    /// [`is_clean_miss`](Self::is_clean_miss) — leaving the scan gate is a far
    /// wider bar that a genuinely safe plan need not reach.
    pub fn capture_radius_m(&self) -> Option<f64> {
        self.nominal_encounter.map(|e| e.capture_radius)
    }

    /// The nominal (un-deflected) b-plane perigee, m — the hit being undone, which
    /// by construction sits inside the capture radius. `None` before a scenario is
    /// installed.
    pub fn nominal_perigee_m(&self) -> Option<f64> {
        self.nominal_encounter.map(|e| e.perigee)
    }

    /// Whether the (expensive) scenario has been built.
    pub fn has_scenario(&self) -> bool {
        self.scenario.is_some()
    }

    /// Whether the five single-term Tier-2 shifts have been measured for the current
    /// scenario — the frontend's cue that the menu's numbers are ready rather than
    /// still `-1`. False right after a build; set once the on-demand preview lands
    /// ([`adopt_tier2_shifts`](Self::adopt_tier2_shifts)).
    pub fn has_tier2_preview(&self) -> bool {
        self.tier2_shifts.is_some()
    }

    /// An `Arc` clone of the built scenario, for handing to the Tier-2 preview
    /// worker ([`measure_tier2_shifts`]). `None` before a scenario is installed.
    /// The clone shares the *exact* scenario the threat was flown in, so the
    /// measured shifts are consistent with the encounter on screen — and it is a
    /// refcount bump, not a rebuild.
    pub fn scenario_arc(&self) -> Option<Arc<RealFieldScenario>> {
        self.scenario.clone()
    }

    /// Adopt Tier-2 shifts measured off-thread by the preview worker — the landing
    /// point for [`measure_tier2_shifts`] run on a [`scenario_arc`](Self::scenario_arc)
    /// clone. Cheap: the expensive propagation already happened on the worker.
    pub fn adopt_tier2_shifts(&mut self, shifts: Tier2Shifts) {
        self.tier2_shifts = Some(shifts);
    }

    /// The measured **shifted nominal perigee** (m) for one Tier-2 term — the
    /// perigee the fixed shipping seed reaches with just that term on. `None` when
    /// the preview was never run, `term` is unknown, or (belt only) the small-body
    /// kernel was absent so the belt shift is genuinely unavailable rather than
    /// zero. The frontend forms the shift as
    /// [`nominal_perigee_m`](Self::nominal_perigee_m) − this.
    ///
    /// `term` is one of `"relativity"`, `"yarkovsky"`, `"belt"`, `"srp"`, `"j2"`.
    pub fn tier2_shifted_perigee_m(&self, term: &str) -> Option<f64> {
        let s = self.tier2_shifts?;
        match term {
            "relativity" => Some(s.relativity_perigee_m),
            "yarkovsky" => Some(s.yarkovsky_perigee_m),
            "belt" => s.belt_perigee_m,
            "srp" => Some(s.srp_perigee_m),
            "j2" => Some(s.j2_perigee_m),
            _ => None,
        }
    }

    /// Heliocentric **ecliptic-J2000** position of NAIF body `naif_id` at
    /// `tdb_seconds` past J2000, in **AU**. `None` if the ephemeris cannot
    /// resolve the body/epoch (out of the kernel span, unknown id). Available as
    /// soon as [`load`](Self::load) succeeds — no scenario required.
    pub fn body_position_ecl_au(&self, naif_id: i32, tdb_seconds: f64) -> Option<Vector3<f64>> {
        let frame = Frame::from_ephem_j2000(naif_id);
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb_seconds);
        self.ephemeris
            .position_km(frame, SUN_J2000, epoch.as_hifitime())
            .ok()
            .map(icrf_km_to_ecliptic_au)
    }

    /// Heliocentric **ecliptic-J2000 AU** from an SSB-relative position in
    /// **metres** (the frame the integrated asteroid `Clock` stores), at `epoch`.
    ///
    /// Subtracts the Sun's SSB position first, so the result lands in the *same*
    /// frame [`body_position_ecl_au`](Self::body_position_ecl_au) puts the planets
    /// in (Sun-relative ecliptic AU); dropping that subtraction would offset the
    /// asteroid from its own drawn orbit by the Sun's barycentric wobble (~1e6 km).
    /// `icrf_km_to_ecliptic_au` wants kilometres, so the metres are scaled down
    /// before the rotation. `None` if the Sun position cannot be resolved.
    fn ssb_m_to_helio_ecl_au(&self, ssb_m: Vector3<f64>, epoch: Epoch) -> Option<Vector3<f64>> {
        let sun_km = self
            .ephemeris
            .position_km(SUN_J2000, SSB_J2000, epoch.as_hifitime())
            .ok()?;
        let helio_km = ssb_m / M_PER_KM - sun_km;
        Some(icrf_km_to_ecliptic_au(helio_km))
    }

    /// The span the threat exists over — `(start, end)` seconds past J2000 — or
    /// `None` before the scenario is built.
    ///
    /// This is the *propagated* span, read from the nominal clock itself rather
    /// than reconstructed from the config, so it cannot drift from what
    /// [`asteroid_position_ecl_au`](Self::asteroid_position_ecl_au) will actually
    /// answer. A display needs it for the same reason it needs
    /// [`usable_span_tdb`](Self::usable_span_tdb): outside this window every
    /// lookup fails, and a failed lookup is `ZERO` — *the Sun's position* in this
    /// heliocentric frame. The threat's window (~12 years) is far narrower than
    /// the kernel's (~300), so the clock clamp does **not** cover it: without this
    /// gate the asteroid would sit on the Sun for most of the scrub range.
    pub fn threat_span_tdb(&self) -> Option<(f64, f64)> {
        Some(self.nominal_clock.as_ref()?.covered_span())
    }

    /// Nominal (un-deflected) threat position, heliocentric **ecliptic AU**, at
    /// `tdb` seconds past J2000 — the asteroid on the solar-system display, in the
    /// same frame as [`body_position_ecl_au`](Self::body_position_ecl_au). `None`
    /// before the scenario is built or for an epoch outside the propagated span
    /// ([`threat_span_tdb`](Self::threat_span_tdb)).
    pub fn asteroid_position_ecl_au(&self, tdb: f64) -> Option<Vector3<f64>> {
        let clock = self.nominal_clock.as_ref()?;
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb);
        let st = clock.state_at(epoch).ok()?;
        self.ssb_m_to_helio_ecl_au(st.position, epoch)
    }

    /// Deflected threat position, heliocentric **ecliptic AU**, at `tdb`.
    ///
    /// Before the plan's deflection epoch the impulse has not been applied, so
    /// this returns the nominal position — otherwise the nudge would appear to act
    /// retroactively. After it, the post-impulse arc. `None` if no plan is set or
    /// the epoch is out of span.
    pub fn deflected_position_ecl_au(&self, tdb: f64) -> Option<Vector3<f64>> {
        let plan = self.plan.as_ref()?;
        let epoch = Epoch::from_tdb_seconds_past_j2000(tdb);
        let st = if tdb < plan.deflection_seconds {
            self.nominal_clock.as_ref()?.state_at(epoch).ok()?
        } else {
            plan.clock.state_at(epoch).ok()?
        };
        self.ssb_m_to_helio_ecl_au(st.position, epoch)
    }

    /// Sample an SSB-position function over `[t0, t1]` at `n` (≥ 2) uniform epochs
    /// and map each into heliocentric ecliptic AU — the shared body of the track
    /// samplers below. Points whose lookup fails are dropped; within a propagated
    /// span (the only way these are called) none do, so the polyline stays whole.
    fn track_ecl_au<F>(&self, n: usize, t0: f64, t1: f64, ssb_at: F) -> Vec<Vector3<f64>>
    where
        F: Fn(Epoch) -> Option<Vector3<f64>>,
    {
        let n = n.max(2);
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            let t = t0 + (t1 - t0) * frac;
            let epoch = Epoch::from_tdb_seconds_past_j2000(t);
            if let Some(au) = ssb_at(epoch).and_then(|p| self.ssb_m_to_helio_ecl_au(p, epoch)) {
                out.push(au);
            }
        }
        out
    }

    /// The nominal threat arc from campaign start to impact as `n` heliocentric
    /// ecliptic-AU points (the orbit polyline). The caller samples this **once**;
    /// the reads are cheap but it walks the whole span. Empty if no scenario.
    pub fn asteroid_track_ecl_au(&self, n: usize) -> Vec<Vector3<f64>> {
        let (Some(clock), Some(sc)) = (self.nominal_clock.as_ref(), self.scenario.as_ref()) else {
            return Vec::new();
        };
        let t0 = sc.epoch0().tdb_seconds_past_j2000();
        let t1 = sc.impact_epoch().tdb_seconds_past_j2000();
        self.track_ecl_au(n, t0, t1, |e| clock.state_at(e).ok().map(|s| s.position))
    }

    /// The deflected threat arc from campaign start to impact as `n` heliocentric
    /// ecliptic-AU points: the nominal track up to the deflection epoch, the
    /// post-impulse arc after it (same guard as
    /// [`deflected_position_ecl_au`](Self::deflected_position_ecl_au)). Empty if
    /// no plan is set.
    pub fn deflected_track_ecl_au(&self, n: usize) -> Vec<Vector3<f64>> {
        let (Some(nom), Some(sc), Some(plan)) = (
            self.nominal_clock.as_ref(),
            self.scenario.as_ref(),
            self.plan.as_ref(),
        ) else {
            return Vec::new();
        };
        let t0 = sc.epoch0().tdb_seconds_past_j2000();
        let t1 = sc.impact_epoch().tdb_seconds_past_j2000();
        self.track_ecl_au(n, t0, t1, |e| {
            let clk = if e.tdb_seconds_past_j2000() < plan.deflection_seconds {
                nom
            } else {
                &plan.clock
            };
            clk.state_at(e).ok().map(|s| s.position)
        })
    }

    /// Commit a deflection plan: an **along-track** impulse of `dv_along_track`
    /// (m/s) applied `lead_seconds` before impact. Recomputes and caches the
    /// deflected arc and its b-plane perigee.
    ///
    /// **Expensive** — it rebuilds the `DeflectionScenario` (re-propagating the
    /// nominal) to find the along-track heading and re-propagate the deflected
    /// arc. Call on a plan change, never per frame. Read the result cheaply via
    /// [`deflected_perigee_m`](Self::deflected_perigee_m) /
    /// [`is_clean_miss`](Self::is_clean_miss) and the deflected position/track.
    ///
    /// The encounter frame is sampled here, from the arc **this call already flew**
    /// — via `frame_from_arcs`, not `frame_from`. That distinction is the whole
    /// reason the core has the split: `frame_from` would fly the identical arc a
    /// second time, doubling this call's ~0.85 s for a picture the propagation in
    /// hand already contains.
    pub fn set_plan(
        &mut self,
        lead_seconds: f64,
        dv_along_track: f64,
    ) -> Result<(), ScenarioError> {
        let sc = self
            .scenario
            .as_ref()
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;
        let nominal_enc = self
            .nominal_encounter
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;
        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-lead_seconds);
        let ds = sc.deflection()?;
        let seed = ds
            .nominal()
            .state_at(deflection_epoch)
            .map_err(|e| ScenarioError::Integration(e.to_string()))?;
        let dir = along_track_unit(seed).ok_or_else(|| {
            ScenarioError::Integration("nominal has zero velocity; no along-track heading".into())
        })?;
        let (clock, encounter) = ds.deflected_trajectory(deflection_epoch, dv_along_track * dir)?;

        // One `DeflectedArc` feeds both the stored geometry and the drawn tracks, so
        // the two are of the same propagation by construction rather than by care.
        let frame = sc.frame_from_arcs(
            ds.nominal(),
            nominal_enc,
            Some(DeflectedArc {
                clock: &clock,
                encounter,
                deflection_epoch,
            }),
            ENCOUNTER_HALF_WINDOW_SECONDS,
            ENCOUNTER_SAMPLES,
        )?;

        self.plan = Some(PlanState {
            deflection_seconds: deflection_epoch.tdb_seconds_past_j2000(),
            clock,
            encounter,
            frame,
        });
        Ok(())
    }

    /// Whether a deflection plan is currently set.
    pub fn has_plan(&self) -> bool {
        self.plan.is_some()
    }

    /// Whether the current plan's deflected pass left the scan gate — a clean,
    /// wide miss (the **success** case), distinct from "no plan" / "solve failed".
    pub fn is_clean_miss(&self) -> bool {
        self.plan.as_ref().is_some_and(|p| p.encounter.is_none())
    }

    /// The deflected b-plane perigee, m — `None` if no plan is set **or** the pass
    /// is a clean miss (use [`is_clean_miss`](Self::is_clean_miss) to tell those two
    /// apart).
    ///
    /// This is the *closest approach* of the pass, and it is **not** the quantity
    /// the hit test compares against the capture radius — see
    /// [`deflected_impact_parameter_m`](Self::deflected_impact_parameter_m). It is
    /// reported because "how close did it actually come" is a real question a
    /// readout may want to answer; it is not the verdict.
    pub fn deflected_perigee_m(&self) -> Option<f64> {
        self.plan
            .as_ref()
            .and_then(|p| p.encounter)
            .map(|e| e.perigee)
    }

    /// The deflected pass's **b-plane impact parameter** `b`, m — the perpendicular
    /// miss of the incoming asymptote from Earth's centre. `None` for no plan or a
    /// clean miss, exactly like [`deflected_perigee_m`](Self::deflected_perigee_m).
    ///
    /// **This is the miss the verdict is made of**, and the one a readout should
    /// print beside the capture radius. `b` pairs with `capture_radius`, and
    /// `perigee` pairs with `earth_radius`; the core proves the two criteria
    /// identical (`geometry.rs`, `hit_criterion_matches_perigee_inside_earth`), but
    /// they are only identical *as pairs*. Comparing a perigee against the capture
    /// radius mixes them and demands ~1.5× more miss than physics does, because the
    /// perigee is already the focused closest approach while the capture radius is
    /// the enlarged target built for the *un*focused asymptotic miss.
    pub fn deflected_impact_parameter_m(&self) -> Option<f64> {
        self.plan
            .as_ref()
            .and_then(|p| p.encounter)
            .map(|e| e.impact_parameter)
    }

    /// The nominal pass's b-plane impact parameter `b`, m — the hit being undone,
    /// which sits inside the capture radius by construction. `None` before a
    /// scenario is installed.
    pub fn nominal_impact_parameter_m(&self) -> Option<f64> {
        self.nominal_encounter.map(|e| e.impact_parameter)
    }

    /// Earth's solid-body radius `R⊕` as the core models it, m — the disc to draw.
    /// The target radius for a *perigee*, never for an impact parameter (that is the
    /// capture radius). `None` before a scenario is installed.
    pub fn earth_radius_m(&self) -> Option<f64> {
        self.nominal_encounter.map(|e| e.earth_radius)
    }

    /// The nominal encounter's hyperbolic excess speed `v_inf`, m/s — the approach
    /// speed "at infinity" that sets how hard Earth's gravity focuses.
    ///
    /// Worth knowing what this is *not*: it is not `ImpactorConfig::v_rel_kms` (18
    /// km/s), which is the speed at the 3000 km impact point, deep in Earth's well.
    /// Stripping the well out leaves `v_inf ≈ 7.63 km/s`, which is what sets the
    /// 1.773 R⊕ capture disc. `None` before a scenario is installed.
    pub fn encounter_v_inf_m_s(&self) -> Option<f64> {
        self.nominal_encounter.map(|e| e.v_inf)
    }

    /// The current plan's deflection epoch, seconds past J2000 (`None` if no plan).
    pub fn plan_deflection_tdb_seconds(&self) -> Option<f64> {
        self.plan.as_ref().map(|p| p.deflection_seconds)
    }

    // --- the b-plane encounter view (3C-2c) ---------------------------------
    //
    // Everything below hands the encounter to the frontend already projected into
    // the display basis (see `bplane_basis`), because choosing that basis is the
    // only judgement involved and it is not one GDScript should be making twice.
    // The frontend receives `(ξ, ζ, s)` kilometres and draws them; it owns no
    // geometry, exactly as `set_plan` left it owning no orbital mechanics.

    /// The b-plane display basis for the built scenario, or `None` before it is.
    fn encounter_basis(&self) -> Option<(Vector3<f64>, Vector3<f64>, Vector3<f64>)> {
        self.nominal_encounter.and_then(|e| bplane_basis(e.s_hat))
    }

    /// The nominal (impact) track through the encounter window, projected into the
    /// b-plane display frame — `(ξ, ζ, s)` km per sample. Empty before the scenario
    /// is built.
    ///
    /// Available with **no plan and no propagation**: this is the pre-plan picture,
    /// the incoming impact the player has to do something about.
    pub fn encounter_nominal_track_km(&self) -> Vec<Vector3<f64>> {
        let Some((basis, frame)) = self.encounter_basis().zip(self.nominal_frame.as_ref()) else {
            return Vec::new();
        };
        frame
            .nominal
            .iter()
            .map(|&g| project_bplane(g, basis))
            .collect()
    }

    /// The deflected track through the encounter window, projected into the same
    /// basis — `(ξ, ζ, s)` km per sample.
    ///
    /// **Empty when there is no plan**, which is not the same as a zero-length
    /// track: there is no deflected pass to draw until the core has propagated one,
    /// and a zeroed track would draw the asteroid straight through Earth's centre.
    pub fn encounter_deflected_track_km(&self) -> Vec<Vector3<f64>> {
        let Some((basis, plan)) = self.encounter_basis().zip(self.plan.as_ref()) else {
            return Vec::new();
        };
        plan.frame
            .deflected
            .iter()
            .map(|&g| project_bplane(g, basis))
            .collect()
    }

    /// The epochs the encounter tracks are sampled at, `(first, last)` seconds past
    /// J2000. Uniformly spaced and shared by both tracks, so a consumer can map a
    /// clock time onto a track index without knowing the window. `None` before the
    /// scenario is built.
    pub fn encounter_sample_span_tdb(&self) -> Option<(f64, f64)> {
        let s = &self.nominal_frame.as_ref()?.sample_seconds;
        Some((*s.first()?, *s.last()?))
    }

    /// The **nominal** b-vector projected into the display frame — `(ξ, ζ, s)` km,
    /// where the asteroid's incoming asymptote pierces the b-plane. `|B|` equals
    /// [`nominal_impact_parameter_m`](Self::nominal_impact_parameter_m), and it lies
    /// inside the capture disc: this is the hit.
    ///
    /// The *sign* of `B` is a convention the core deliberately leaves unpinned
    /// (`geometry.rs` §10.8), so which side of the disc this point lands on is
    /// cosmetic — its distance from the centre, which is what the verdict reads, is
    /// not. `None` before the scenario is built.
    pub fn nominal_b_point_km(&self) -> Option<Vector3<f64>> {
        let basis = self.encounter_basis()?;
        Some(project_bplane(self.nominal_encounter?.b_vector, basis))
    }

    /// The **deflected** b-vector in the display frame — `(ξ, ζ, s)` km, at exactly
    /// `|B| =` [`deflected_impact_parameter_m`](Self::deflected_impact_parameter_m)
    /// from the origin. `None` for no plan or a clean miss (there is no finite
    /// b-plane point once the pass has left the scan gate). Same unpinned-sign
    /// caveat as [`nominal_b_point_km`](Self::nominal_b_point_km).
    ///
    /// **Why this one is rescaled and the nominal is not.** Each pass has its own
    /// b-plane, perpendicular to *its own* `Ŝ`. The frame here is the nominal's, so
    /// the nominal's `B` lies in it exactly (`B ⊥ Ŝ` by construction, `s = 0`) while
    /// the deflected `B` — perpendicular to the *deflected* asymptote — does not,
    /// and a raw projection would plot it at `√(|B|² − s²)`, slightly inside its own
    /// stated `|B|`. The verdict is the scalar `|B|` against the capture radius, so
    /// a mark drawn even slightly off that radius could sit inside the disc while
    /// the panel reads MISS: the picture contradicting the physics, which is the
    /// failure this whole view exists to end. `|B|` is pinned and the *direction* is
    /// an unpinned display convention, so the honest render keeps the magnitude
    /// exact and takes only the bearing from the projection.
    ///
    /// Measured, this is a guarantee rather than a visible change: a small
    /// along-track nudge moves *where* the rock arrives enormously (years of
    /// leverage) but barely rotates *how* it approaches, so the two asymptotes sit
    /// within ~0.01–0.2° and the raw gap is under 0.01%. Being right by construction
    /// beats being right by a coincidence nobody re-measures.
    pub fn deflected_b_point_km(&self) -> Option<Vector3<f64>> {
        let basis = self.encounter_basis()?;
        let enc = self.plan.as_ref()?.encounter?;
        let p = project_bplane(enc.b_vector, basis);
        let transverse = Vector3::new(p.x, p.y, 0.0);
        let r = transverse.norm();
        if r < f64::EPSILON {
            // B is (absurdly) along the nominal asymptote: no bearing to draw it on.
            return None;
        }
        Some(transverse * (enc.impact_parameter / M_PER_KM / r))
    }

    /// The minimum along-track Δv (m/s) that lifts the b-plane perigee to
    /// `target_perigee_m` when applied `lead_seconds` before impact — one point
    /// of the headline curve. Errors if the scenario is not built yet.
    pub fn required_dv_along_track(
        &self,
        lead_seconds: f64,
        target_perigee_m: f64,
    ) -> Result<f64, ScenarioError> {
        let sc = self
            .scenario
            .as_ref()
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;
        let ds = sc.deflection()?;
        let deflection_epoch = sc.impact_epoch().shifted_by_seconds(-lead_seconds);
        Ok(ds.required_dv_along_track(deflection_epoch, target_perigee_m, DvSolveTol::default())?)
    }

    /// Heliocentric semi-major axis of the threat, m (0 if no scenario).
    pub fn semi_major_axis_m(&self) -> f64 {
        self.scenario.as_ref().map_or(0.0, |s| s.semi_major_axis_m)
    }

    /// Heliocentric orbital period of the threat, seconds (0 if no scenario).
    pub fn period_seconds(&self) -> f64 {
        self.scenario.as_ref().map_or(0.0, |s| s.period_seconds)
    }

    /// Impact epoch, seconds past J2000 (0 if no scenario).
    pub fn impact_tdb_seconds(&self) -> f64 {
        self.scenario
            .as_ref()
            .map_or(0.0, |s| s.impact_epoch().tdb_seconds_past_j2000())
    }

    /// Campaign-start epoch, seconds past J2000 (0 if no scenario).
    pub fn epoch0_tdb_seconds(&self) -> f64 {
        self.scenario
            .as_ref()
            .map_or(0.0, |s| s.epoch0().tdb_seconds_past_j2000())
    }

    // --- Orrery catalog (the multi-body, long-span, scrubbable display) --------

    /// Add a **synthetic designer body** to the orrery catalog and return its
    /// index. The orbit is given by classical Keplerian `elements` referred to the
    /// **ecliptic** (the plane the display and a human designer reason in), valid
    /// at `epoch0`; the body is then integrated **once** through the scenario's
    /// validated Tier-1 field into a dense-output [`Clock`] spanning
    /// `n_snapshots · cadence_seconds` from `epoch0` (sign of the cadence sets the
    /// direction — a forward span for a body seeded at the display's start epoch).
    ///
    /// Requires [`build_scenario`](Self::build_scenario) first (the field lives on
    /// the scenario). The seed is built in the integration frame: element→state
    /// about the Sun (heliocentric, ecliptic), rotate ecliptic→ICRF, add the Sun's
    /// SSB state — the exact inverse of the read path, so a query back at `epoch0`
    /// recovers the authored position.
    ///
    /// **Cost:** one N-body integration over the whole span (seconds for a
    /// multi-decade comet). Call at load, not per frame; reads are cheap after.
    pub fn add_synthetic_body(
        &mut self,
        name: &str,
        kind: &str,
        elements: OrbitalElements,
        epoch0: Epoch,
        cadence_seconds: f64,
        n_snapshots: u32,
    ) -> Result<usize, ScenarioError> {
        let sc = self
            .scenario
            .as_ref()
            .ok_or_else(|| ScenarioError::NominalNotAHit("scenario not built".into()))?;

        let body = seed_orrery_body(
            &self.ephemeris,
            sc,
            name,
            kind,
            elements,
            epoch0,
            cadence_seconds,
            n_snapshots,
        )?;
        self.bodies.push(body);
        Ok(self.bodies.len() - 1)
    }

    /// Number of bodies in the orrery catalog.
    pub fn catalog_count(&self) -> usize {
        self.bodies.len()
    }

    /// Display label of catalog body `i` (`None` if out of range).
    pub fn catalog_name(&self, i: usize) -> Option<&str> {
        self.bodies.get(i).map(|b| b.name.as_str())
    }

    /// Coarse class of catalog body `i` (`"asteroid"`/`"comet"`/…; `None` if OOR).
    pub fn catalog_kind(&self, i: usize) -> Option<&str> {
        self.bodies.get(i).map(|b| b.kind.as_str())
    }

    /// Where catalog body `i`'s positions come from: `"integrated"` (our physics,
    /// in our field) or `"sampled"` (JPL's, read from a Horizons table). `None` if
    /// `i` is out of range.
    ///
    /// Exists so the frontend can *label* the distinction rather than quietly
    /// drawing two kinds of physics in one colour. That is not a style preference
    /// here: a display-grade Kepler propagator drawn beside the real integrated
    /// threat, with nothing marking which was which, is the specific mistake this
    /// project spent a phase deleting.
    pub fn catalog_provenance(&self, i: usize) -> Option<&'static str> {
        self.bodies.get(i).map(|b| match b.trajectory {
            Trajectory::Integrated(_) => "integrated",
            Trajectory::Sampled(_) => "sampled",
        })
    }

    /// The covered span of catalog body `i` as `(lo, hi)` seconds past J2000 —
    /// the frontend clamps/hides the body outside this (the reverse/long scrub
    /// exposes bodies with a bounded arc). `None` if `i` is out of range.
    ///
    /// **Both variants are bounded, for different reasons**: an integrated body
    /// covers what was propagated, a sampled one covers the years its table was
    /// fetched for. Neither is the clock's span, and outside either the position
    /// query returns `None` — which the caller must honour rather than draw, since
    /// a zeroed position is the Sun.
    pub fn catalog_span_tdb(&self, i: usize) -> Option<(f64, f64)> {
        self.bodies.get(i).map(|b| match &b.trajectory {
            Trajectory::Integrated(c) => c.covered_span(),
            Trajectory::Sampled(n) => n.span_tdb(),
        })
    }

    /// Position of catalog body `i` at `tdb`, heliocentric **ecliptic AU** — the
    /// same display frame as the planets and the threat. `None` if `i` is out of
    /// range or `tdb` is outside the body's span (the frontend uses
    /// [`catalog_span_tdb`](Self::catalog_span_tdb) to know which).
    pub fn catalog_position_ecl_au(&self, i: usize, tdb: f64) -> Option<Vector3<f64>> {
        let b = self.bodies.get(i)?;
        self.catalog_body_helio_ecl_au(b, Epoch::from_tdb_seconds_past_j2000(tdb))
    }

    /// **The one place the two trajectory frames are reconciled.** Integrated
    /// bodies come back in SSB metres and have the Sun's barycentric position
    /// subtracted; sampled bodies are already Sun-relative and must not be
    /// shifted again. Both then take the same ICRF→ecliptic rotation the planets
    /// take, so everything on screen lands in one frame.
    ///
    /// Doing this once, here, is the point: the wrong branch is a ~10⁶ km offset
    /// that looks like a rendering nudge, and two copies of this logic is two
    /// chances to get it wrong.
    fn catalog_body_helio_ecl_au(&self, b: &OrreryBody, epoch: Epoch) -> Option<Vector3<f64>> {
        match &b.trajectory {
            Trajectory::Integrated(c) => {
                self.ssb_m_to_helio_ecl_au(c.state_at(epoch).ok()?.position, epoch)
            }
            Trajectory::Sampled(n) => {
                let helio_m = n.helio_state_at(epoch.tdb_seconds_past_j2000())?.position;
                Some(icrf_km_to_ecliptic_au(helio_m / M_PER_KM))
            }
        }
    }

    /// One orbital period of catalog body `i`'s trajectory, seconds — or the whole
    /// covered span when a period is not meaningful (an integrated body, or a
    /// sampled one on an unbound arc). `None` if `i` is out of range.
    ///
    /// The orbit *line* wants one lap: a sampled NEO's table is decades long but
    /// its orbit is ~a year, and a polyline over the whole table is dozens of
    /// precessing laps drawn on top of each other. This says how much to draw.
    pub fn catalog_orbit_period_seconds(&self, i: usize) -> Option<f64> {
        match &self.bodies.get(i)?.trajectory {
            // The comet's span already *is* one authored period, and an integrated
            // body has no closed-form period here — draw the whole span.
            Trajectory::Integrated(c) => {
                let (lo, hi) = c.covered_span();
                Some(hi - lo)
            }
            Trajectory::Sampled(n) => {
                let (lo, hi) = n.span_tdb();
                Some(n.orbital_period_seconds().unwrap_or(hi - lo))
            }
        }
    }

    /// Catalog body `i`'s trajectory as `n` heliocentric ecliptic-AU points across
    /// its whole covered span — the orbit polyline. Sampled **once**. Empty if
    /// `i` is out of range.
    ///
    /// Goes through [`catalog_body_helio_ecl_au`](Self::catalog_body_helio_ecl_au)
    /// rather than the SSB-only `track_ecl_au` helper the threat paths use,
    /// because a sampled body's states are not SSB and that helper would subtract
    /// the Sun from a position the Sun had already been subtracted from.
    pub fn catalog_track_ecl_au(&self, i: usize, n: usize) -> Vec<Vector3<f64>> {
        match self.catalog_span_tdb(i) {
            Some((t0, t1)) => self.catalog_track_window_ecl_au(i, t0, t1, n),
            None => Vec::new(),
        }
    }

    /// Catalog body `i`'s trajectory as `n` points across `[t0_tdb, t1_tdb]` — the
    /// orbit polyline over an arbitrary window, so a decades-long sampled body can
    /// draw a single lap rather than dozens overplotted. Points outside the body's
    /// span are dropped (never drawn at the Sun), so a window straying past an edge
    /// yields a short arc, not a spray of origin points. Empty if `i` is invalid.
    pub fn catalog_track_window_ecl_au(
        &self,
        i: usize,
        t0_tdb: f64,
        t1_tdb: f64,
        n: usize,
    ) -> Vec<Vector3<f64>> {
        let Some(b) = self.bodies.get(i) else {
            return Vec::new();
        };
        let n = n.max(2);
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let t = t0_tdb + (t1_tdb - t0_tdb) * (k as f64 / (n - 1) as f64);
            if let Some(au) =
                self.catalog_body_helio_ecl_au(b, Epoch::from_tdb_seconds_past_j2000(t))
            {
                out.push(au);
            }
        }
        out
    }
}

// --- The porkchop / deliverability view --------------------------------------
//
// The frontend half of `core::mission` (HANDOFF §8): a launch × arrival grid the
// operator can read, and one selected cell verified in the full field. Split by
// cost exactly as the core layer is — a cheap vehicle-independent grid built once
// on a worker, and a single expensive propagation fired on demand.

/// Radius of the synthetic threat, metres.
///
/// **The frontend must not invent a third rock.** The scenario already commits to
/// a body through [`SrpParams::sub_km_rock`], which folds 150 m at 2000 kg/m³ into
/// an area-to-mass ratio and keeps the dimensions as locals — so a delivery layer
/// that wants the *mass* has no choice but to restate them. Naming them here (and
/// pinning them to the SRP default by [`threat_body_matches_the_srp_default`])
/// makes a careless edit to either side fail loudly, the same treatment
/// `SB441_BODIES` gets against the core's GM table.
pub const THREAT_RADIUS_M: f64 = 150.0;

/// Bulk density of the synthetic threat, kg/m³ — see [`THREAT_RADIUS_M`].
pub const THREAT_DENSITY_KG_M3: f64 = 2000.0;

/// Momentum-enhancement factor `β` for the kinetic impactor. DART measured ≈ 3.6
/// at Dimorphos; one named constant so the grid's delivered-Δv column and the
/// on-demand full-field verify can never be reading different physics.
pub const IMPACTOR_BETA: f64 = 3.6;

/// Mass of the synthetic threat, kg — `4/3·π·r³·ρ` from [`THREAT_RADIUS_M`] and
/// [`THREAT_DENSITY_KG_M3`]. ≈ 2.83e10 kg.
pub fn threat_mass_kg() -> f64 {
    (4.0 / 3.0) * std::f64::consts::PI * THREAT_RADIUS_M.powi(3) * THREAT_DENSITY_KG_M3
}

/// The launcher at `index` in the core's canonical
/// [`LAUNCH_VEHICLES`](asteroid_core::launch_vehicle::LAUNCH_VEHICLES) table, or
/// `None` past the end. The frontend cycles this table by index; the table itself
/// stays in the core so the display and the physics can never offer different
/// launchers.
pub fn launch_vehicle(index: usize) -> Option<&'static LaunchVehicle> {
    LAUNCH_VEHICLES.get(index)
}

/// How many launchers the frontend can cycle through.
pub fn launch_vehicle_count() -> usize {
    LAUNCH_VEHICLES.len()
}

/// Where the launch axis stops, as a fraction of the campaign span (`impact −
/// epoch0`). Launching later than this leaves no room for a transfer *and* the
/// lead the deflection needs afterwards.
const LAUNCH_AXIS_END_FRACTION: f64 = 0.70;

/// Where the arrival axis starts, as a fraction of the campaign span — the
/// earliest interception worth plotting, a minimum cruise past `epoch0`.
const ARRIVAL_AXIS_START_FRACTION: f64 = 0.10;

/// Where the arrival axis stops, as a fraction of the campaign span. Deliberately
/// short of impact: an intercept in the last months deflects almost nothing
/// however well aimed (the §5 lever is lead time), so plotting up to the impact
/// itself would spend a third of the frame on windows that cannot work.
const ARRIVAL_AXIS_END_FRACTION: f64 = 0.92;

/// Shortest transfer the grid will consider, days. Below this the Lambert arc is a
/// near-radial sprint no launcher reaches; the cells are blanked by the grid's own
/// `min_tof` guard rather than filled with astronomical `C3`.
const MIN_TOF_DAYS: f64 = 90.0;

/// How many complete laps of the Sun a cell's transfer may make, in the shipping
/// grid.
///
/// **Not zero, and that is a correctness choice rather than a nicety.** The
/// campaign's times of flight run for years, and over spans that long the direct
/// arc is the slow, ruinously expensive conic: measured at 2.6 yr, direct
/// `C3 = 933 km²/s²` (no launcher on the list reaches it) against `55` for the
/// lapping transfer. A `max_revolutions = 0` grid would render its entire
/// long-time-of-flight half as an infeasible wall — wrong about real mission
/// design, not merely conservative.
///
/// **Two, not one, and that was measured rather than assumed.** On a 24×24 grid
/// over the default campaign — 379 real transfers, and the count of them Falcon
/// Heavy (expendable) can actually reach:
///
/// | laps | reachable | cheapest `C3` | cost |
/// |-----:|----------:|--------------:|-----:|
/// | `0`  | 21        | 1.53 km²/s²   | 4.8 µs/cell |
/// | `1`  | 56        | 0.34          | 47.8 |
/// | `2`  | **95**    | 0.34          | 69.0 |
/// | `3`  | 129       | 0.29          | 106.8 |
///
/// A direct-only grid therefore shows an operator **under a quarter** of the
/// missions that exist. Two laps more than quadruples that and keeps a 120×120
/// grid near a second on a worker.
///
/// Note where the cheapest column lands: allowing even one lap takes the grid to
/// `C3 = 0.34`, **below the first knot of three of the five vehicle tables**. That
/// is what made [`LaunchVehicle::payload_kg`]'s old fail-closed-at-both-ends
/// behaviour visible as a bug, and it is asserted in
/// [`revolutions_open_windows_and_what_they_cost`] rather than left as a remark.
///
/// The table is also the argument for *stopping* at two rather than going higher:
/// laps keep opening windows (they always will — at long times of flight a tighter
/// orbit that laps is simply cheaper than crawling round on a huge one), so there
/// is no natural knee to find, only a cost that keeps doubling for missions whose
/// cruise grows by a full solar orbit each step. Reproduced by
/// [`revolutions_open_windows_and_what_they_cost`], which will print the current
/// numbers if the scenario or the vehicle tables change.
pub const DEFAULT_MAX_REVOLUTIONS: u32 = 2;

/// A built porkchop grid, ready for the frontend to read column-wise.
///
/// Owns the core [`Porkchop`] and nothing else: every accessor here is a
/// projection of cells the core already solved, so the vehicle-independence the
/// core layer bought survives to the display — switching launcher re-reads
/// [`payload_kg_flat`](Self::payload_kg_flat), it never re-solves Lambert.
pub struct PorkchopView {
    grid: Porkchop,
}

/// The outcome of verifying one selected cell in the full `n`-body field.
///
/// An enum rather than a struct of sentinels because the three outcomes are
/// genuinely different states and this project has already been bitten by
/// collapsing them: a clean miss is the *best* result, and if it shared a `-1`
/// with "not verified yet" the frontend would print the safest plan as a failure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CellVerdict {
    /// The deflected pass left the close-approach scan gate entirely — no Earth
    /// encounter at all. The best possible outcome, and it has no b-plane numbers
    /// *because there is no encounter to reduce*, not because they are missing.
    CleanMiss,
    /// The deflected pass still passes Earth; here is its b-plane geometry.
    Encounter {
        /// Impact parameter `|B|`, m — the miss distance the hit test judges.
        impact_parameter_m: f64,
        /// Focused capture radius `b_capture`, m — the bar `|B|` is measured
        /// against. **These two pair with each other**; the perigee pairs with
        /// R⊕. Mixing the pairs is the 3C-2c bug (~1.5× too strict).
        capture_radius_m: f64,
        /// Perigee `r_p`, m — the focused closest approach, reported alongside
        /// because it is what "how close did it come" means to a reader.
        perigee_m: f64,
        /// Earth's solid radius R⊕, m — the perigee's partner.
        earth_radius_m: f64,
        /// The core's own [`BPlaneEncounter::is_hit`]: `|B| ≤ b_capture`.
        is_hit: bool,
    },
    /// The deflected pass is not hyperbolic about Earth — a dead-centre capture.
    /// A hit with no b-plane reduction available, distinct from a hit that has one.
    NotHyperbolic,
}

/// Everything the readout panel shows for one cell, marshalled in one call so the
/// display cannot assemble a row out of two different cells.
#[derive(Debug, Clone, Copy)]
pub struct CellDetail {
    /// Launch epoch, TDB seconds past J2000.
    pub launch_tdb: f64,
    /// Arrival epoch, TDB seconds past J2000.
    pub arrival_tdb: f64,
    /// Time of flight, days.
    pub tof_days: f64,
    /// Departure `C3`, km²/s².
    pub c3_km2_s2: f64,
    /// Arrival relative speed, m/s.
    pub arrival_v_rel_ms: f64,
    /// Along-track projection of the impact, m/s — **signed**. Negative is a
    /// retrograde, orbit-shrinking push: a real lever, not bad aim.
    pub along_track_proj_ms: f64,
    /// Complete laps of the Sun this transfer makes (0 = direct).
    pub revolutions: u32,
    /// Deliverable impactor mass at this `C3` for the selected vehicle, kg.
    /// `0` means this launcher cannot reach this launch energy.
    pub payload_kg: f64,
    /// The along-track Δv that delivered mass imparts, m/s (signed).
    pub along_track_dv_ms: f64,
}

impl PorkchopView {
    /// Build the grid over a scenario — the worker-thread entry point.
    ///
    /// Both axes are derived from the scenario's own campaign (`epoch0` →
    /// `impact_epoch`) rather than from literals, because an arrival outside the
    /// propagated nominal span is a `NoTransfer` *by construction*
    /// (`mission::porkchop_grid`): hardcoded dates would ship a half-blank grid
    /// with no way to tell physics from an axis bug.
    /// The shipping grid: [`build_with_revolutions`](Self::build_with_revolutions)
    /// at [`DEFAULT_MAX_REVOLUTIONS`].
    pub fn build(
        scenario: &RealFieldScenario,
        launch_samples: usize,
        arrival_samples: usize,
    ) -> Result<Self, ScenarioError> {
        Self::build_with_revolutions(
            scenario,
            launch_samples,
            arrival_samples,
            DEFAULT_MAX_REVOLUTIONS,
        )
    }

    /// As [`build`](Self::build), with the lap budget explicit — so how many laps
    /// the shipping grid allows can be *measured* against the alternatives rather
    /// than asserted.
    pub fn build_with_revolutions(
        scenario: &RealFieldScenario,
        launch_samples: usize,
        arrival_samples: usize,
        max_revolutions: u32,
    ) -> Result<Self, ScenarioError> {
        let launch_samples = launch_samples.max(2);
        let arrival_samples = arrival_samples.max(2);
        let t0 = scenario.epoch0().tdb_seconds_past_j2000();
        let span = scenario.impact_epoch().tdb_seconds_past_j2000() - t0;

        let axis = |lo_frac: f64, hi_frac: f64, n: usize| -> Vec<Epoch> {
            (0..n)
                .map(|k| {
                    let f = lo_frac + (hi_frac - lo_frac) * (k as f64 / (n - 1) as f64);
                    Epoch::from_tdb_seconds_past_j2000(t0 + f * span)
                })
                .collect()
        };
        let launches = axis(0.0, LAUNCH_AXIS_END_FRACTION, launch_samples);
        let arrivals = axis(
            ARRIVAL_AXIS_START_FRACTION,
            ARRIVAL_AXIS_END_FRACTION,
            arrival_samples,
        );

        let grid = porkchop_grid(
            scenario,
            &launches,
            &arrivals,
            MIN_TOF_DAYS * 86_400.0,
            /*prograde*/ true,
            max_revolutions,
        )
        .map_err(|e| ScenarioError::Integration(format!("porkchop grid: {e}")))?;

        Ok(Self { grid })
    }

    /// Number of launch epochs (the grid's first axis / row count).
    pub fn launch_count(&self) -> usize {
        self.grid.launch_epochs.len()
    }

    /// Number of arrival epochs (the grid's second axis / column count).
    pub fn arrival_count(&self) -> usize {
        self.grid.arrival_epochs.len()
    }

    /// The launch axis, TDB seconds past J2000.
    pub fn launch_tdb(&self) -> Vec<f64> {
        self.grid
            .launch_epochs
            .iter()
            .map(|e| e.tdb_seconds_past_j2000())
            .collect()
    }

    /// The arrival axis, TDB seconds past J2000.
    pub fn arrival_tdb(&self) -> Vec<f64> {
        self.grid
            .arrival_epochs
            .iter()
            .map(|e| e.tdb_seconds_past_j2000())
            .collect()
    }

    /// The metrics at `[launch][arrival]`, or `None` for a `NoTransfer` cell.
    pub fn metrics_at(&self, i: usize, j: usize) -> Option<TransferMetrics> {
        match self.grid.cells.get(i)?.get(j)? {
            PorkchopCell::Transfer(m) => Some(*m),
            PorkchopCell::NoTransfer => None,
        }
    }

    /// Departure `C3` per cell, km²/s², row-major `[launch][arrival]`.
    ///
    /// **`-1.0` marks a `NoTransfer` cell** — no trajectory exists at any allowed
    /// revolution count. A negative `C3` is physically impossible, so this is an
    /// unambiguous sentinel and no `NaN` ever crosses the FFI boundary (a `NaN` in
    /// a packed float would poison every min/max the display takes over the grid).
    ///
    /// This array is the **single authority on emptiness**: the other per-cell
    /// arrays carry ordinary zeros where a cell is blank, so a reader that checked
    /// them instead would confuse "no transfer" with "a transfer that projects to
    /// nothing". Those are the two blanks the display must keep apart, along with
    /// the third — a real transfer this *launcher* cannot reach
    /// ([`payload_kg_flat`](Self::payload_kg_flat) `== 0`).
    pub fn c3_flat(&self) -> Vec<f64> {
        self.map_flat(|m| m.c3_km2_s2, -1.0)
    }

    /// Arrival relative speed per cell, m/s (`0` where blank).
    pub fn arrival_v_rel_flat(&self) -> Vec<f64> {
        self.map_flat(|m| m.arrival_v_rel_ms, 0.0)
    }

    /// Signed along-track projection per cell, m/s (`0` where blank).
    pub fn along_track_flat(&self) -> Vec<f64> {
        self.map_flat(|m| m.along_track_proj_ms, 0.0)
    }

    /// Complete laps per cell; `-1` where blank. A lapping cell is a *different
    /// cruise*, not just a different number, which is why the grid carries it.
    pub fn revolutions_flat(&self) -> Vec<i32> {
        let mut out = Vec::with_capacity(self.launch_count() * self.arrival_count());
        for row in &self.grid.cells {
            for cell in row {
                out.push(match cell {
                    PorkchopCell::Transfer(m) => m.revolutions as i32,
                    PorkchopCell::NoTransfer => -1,
                });
            }
        }
        out
    }

    /// Deliverable impactor mass per cell for `vehicle`, kg — `0` where the
    /// launcher cannot reach that `C3`, **and also `0` where no transfer exists**.
    /// Read against [`c3_flat`](Self::c3_flat) to tell the two apart.
    pub fn payload_kg_flat(&self, vehicle: &LaunchVehicle) -> Vec<f64> {
        self.map_flat(
            |m| cell_delivery(&m, vehicle, IMPACTOR_BETA, threat_mass_kg()).payload_kg,
            0.0,
        )
    }

    /// The along-track Δv the delivered mass imparts per cell, m/s (signed;
    /// `0` where infeasible or blank).
    pub fn along_track_dv_flat(&self, vehicle: &LaunchVehicle) -> Vec<f64> {
        self.map_flat(
            |m| cell_delivery(&m, vehicle, IMPACTOR_BETA, threat_mass_kg()).along_track_dv_ms,
            0.0,
        )
    }

    /// Everything the readout shows for one cell, or `None` if the indices are out
    /// of range or the cell holds no transfer.
    pub fn detail(&self, i: usize, j: usize, vehicle: &LaunchVehicle) -> Option<CellDetail> {
        let m = self.metrics_at(i, j)?;
        let launch_tdb = self.grid.launch_epochs[i].tdb_seconds_past_j2000();
        let arrival_tdb = self.grid.arrival_epochs[j].tdb_seconds_past_j2000();
        let d = cell_delivery(&m, vehicle, IMPACTOR_BETA, threat_mass_kg());
        Some(CellDetail {
            launch_tdb,
            arrival_tdb,
            tof_days: (arrival_tdb - launch_tdb) / 86_400.0,
            c3_km2_s2: m.c3_km2_s2,
            arrival_v_rel_ms: m.arrival_v_rel_ms,
            along_track_proj_ms: m.along_track_proj_ms,
            revolutions: m.revolutions,
            payload_kg: d.payload_kg,
            along_track_dv_ms: d.along_track_dv_ms,
        })
    }

    fn map_flat(&self, f: impl Fn(TransferMetrics) -> f64, blank: f64) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.launch_count() * self.arrival_count());
        for row in &self.grid.cells {
            for cell in row {
                out.push(match cell {
                    PorkchopCell::Transfer(m) => f(*m),
                    PorkchopCell::NoTransfer => blank,
                });
            }
        }
        out
    }
}

/// Re-propagate the asteroid in the **full `n`-body field** after one cell's real
/// vector impulse and reduce the Earth encounter it produces — the on-demand half
/// of the layer, ~one propagation, fired per *selected* cell and never across the
/// grid.
///
/// `impactor_mass_kg` is meant to be the selected launcher's deliverable mass at
/// that cell's `C3`, which is what makes this the honest question: not "would some
/// impulse work" but "does *this launcher* through *this window* work".
///
/// **Cost is cell-dependent: measured 5.8 s at a late arrival, 18.2 s at an early
/// one**, because the propagation re-flies from arrival to the encounter and the
/// arrival axis spans 0.10–0.92 of a ~12 yr campaign. (An earlier note here said
/// "~1 s"; that was never measured and is wrong by 6–18×. It matters because it is
/// the unit the required-mass solve is priced in — see [`required_cell_mass`].)
pub fn verify_porkchop_cell(
    scenario: &RealFieldScenario,
    arrival_tdb: f64,
    metrics: &TransferMetrics,
    impactor_mass_kg: f64,
) -> Result<CellVerdict, ScenarioError> {
    let ds = scenario.deflection()?;
    let arrival = Epoch::from_tdb_seconds_past_j2000(arrival_tdb);
    match verify_cell(
        &ds,
        arrival,
        metrics,
        IMPACTOR_BETA,
        impactor_mass_kg,
        threat_mass_kg(),
    ) {
        Ok(Some(bp)) => Ok(CellVerdict::Encounter {
            impact_parameter_m: bp.impact_parameter,
            capture_radius_m: bp.capture_radius,
            perigee_m: bp.perigee,
            earth_radius_m: bp.earth_radius,
            is_hit: bp.is_hit(),
        }),
        Ok(None) => Ok(CellVerdict::CleanMiss),
        Err(DeflectionError::Geometry(asteroid_core::geometry::GeometryError::NotHyperbolic {
            ..
        })) => Ok(CellVerdict::NotHyperbolic),
        Err(e) => Err(ScenarioError::Integration(format!("cell verify: {e}"))),
    }
}

/// The heaviest mass any launcher on the core's table can deliver, kg — each
/// vehicle at its own cheapest tabulated `C3`.
///
/// Derived from [`LAUNCH_VEHICLES`] rather than written down, so it cannot drift
/// from the AMAT/LSP tables the rest of the layer reads. Used to size the
/// required-mass solve's cap ([`mass_solve_cap_kg`]) in units a reader can convert:
/// a requirement quoted as *n* × this is *n* launches' worth of the best rocket
/// there is.
pub fn heaviest_deliverable_kg() -> f64 {
    LAUNCH_VEHICLES
        .iter()
        .map(|v| v.payload_kg(v.min_c3_km2_s2()))
        .fold(0.0, f64::max)
}

/// Where the required-mass bracket gives up, kg — **one hundred** of the best
/// launch there is.
///
/// Sized off [`heaviest_deliverable_kg`] rather than a round number, because that
/// is what makes [`MassSolveOutcome::InfeasibleAtCap`] *readable*: hitting it means
/// "not this window, not with a hundred of the best launcher there is", which is a
/// statement about the mission rather than about a solver parameter.
///
/// A cap is mandatory, not a nicety — it is the degenerate-direction guard the core
/// solver was built with from day one. A window whose arrival geometry projects
/// poorly onto the track (`v_rel ⊥ v̂_ast`) is deflected by *no* deliverable mass,
/// and an uncapped bisection would double forever chasing it.
///
/// A hundred rather than ten, and that was measured. Ten (≈ 64 t) puts the cap
/// *below* the requirement of every window in the shipping grid, so every cell
/// would return `InfeasibleAtCap` and the view would never show a mass at all — the
/// number this whole feature exists to print. The best-coupled early window
/// measured **157 t**, which a hundred brackets and ten does not.
pub fn mass_solve_cap_kg() -> f64 {
    100.0 * heaviest_deliverable_kg()
}

/// Where the required-mass bracket starts, kg — **one** best launch.
///
/// A meaningful anchor rather than an arbitrarily tiny number, and that is worth
/// probes: the bracket doubles from here, so starting at 1 kg would spend a dozen
/// full-field propagations climbing to the mission scale before learning anything.
/// Starting at what the best rocket actually lifts puts the first probe inside the
/// range the answer lives in.
///
/// **Safe only because the answer is seed-independent.** The core solver used to
/// return the seed verbatim when the seed already cleared the target; it now
/// brackets downward instead (`required_impactor_mass`), so a well-chosen seed buys
/// speed and cannot change the reported physics. Choosing this seed before that fix
/// would have quietly made the readout a function of the launcher table.
pub fn mass_solve_seed_kg() -> f64 {
    heaviest_deliverable_kg()
}

/// Relative width the required-mass bisection stops at — 5 %.
///
/// Sized to the *readout*, not to the solver: the panel prints a mass rounded to
/// three significant figures beside a ratio rounded to a whole number, and neither
/// can show 5 %. The core's old hardcoded 1e-4 cost **seven extra full-field
/// propagations** (~2 minutes on an early-arrival cell) to compute digits nothing
/// displays.
pub const MASS_SOLVE_TOL: f64 = 0.05;

/// Solve for the impactor mass one launch window needs to reach the campaign's
/// safe-perigee target ([`SAFE_PERIGEE_TARGET_M`]) — the on-demand answer behind
/// the launch-window map's `[M]` key.
///
/// **Vehicle-independent, on purpose.** The question is what the *window* costs,
/// not what a particular rocket happens to carry; the frontend divides this by the
/// selected launcher's payload to get the ratio. That independence is only real
/// because the core solver brackets from a fixed seed — seeding from the launcher's
/// payload would make the answer change when you pressed `[L]`.
///
/// **Expensive, and every constant above exists because it was measured.** Each
/// probe is a full-field re-propagation from arrival to the encounter, and the probe
/// cost is *cell-dependent* — **18.2 s** for an early-arrival window (10.8 yr of
/// cruise to re-fly) against **5.8 s** for a late one (3.2 yr) — so what decides the
/// wall clock is how many probes the bracket takes.
///
/// Measured end to end on the shipping grid: **46 s** for the best-coupled window
/// (feasible at 20 232 kg) and **31 s** for the worst-coupled late one (over the
/// cap). Early-arrival cells far from the seed are the slow tail, ~3 min. The same
/// order as the Tier-2 preview's ~80 s, and like it: strictly on a worker, with the
/// frontend saying it is running.
///
/// **Both measured cells bracket *upward*, so that range does not cover a window
/// cheaper than the seed.** Such a window halves down instead
/// (`required_impactor_mass`'s downward bracket), paying the same per-probe cost per
/// halving with only [`MIN_BRACKET_MASS_KG`](asteroid_core::mission) — one gram —
/// bounding it. No cell in the shipping grid does this: the seed is one best launch
/// and the cheapest requirement found is 1.4× that. A grid whose windows got much
/// easier would want the seed lowered to match, and would be timing a path nothing
/// has yet measured.
///
/// The naive parameterisation this replaced — a 100 kg seed, the core's old
/// hardcoded 1e-4 tolerance, a 1e9 cap — took **455 s** for the same answer, and it
/// is worth naming which knob bought what: the seed removed ~11 doublings, the
/// tolerance ~7 bisections, and the cap is what keeps the *unreachable* windows from
/// climbing to 1e9 before admitting it.
///
/// The cap direction matters in the other sense too. Sized at ten launches instead
/// of a hundred, it would sit below every window's requirement, so every cell would
/// read "over the cap" and the number this feature exists to print would never
/// appear.
pub fn required_cell_mass(
    scenario: &RealFieldScenario,
    arrival_tdb: f64,
    metrics: &TransferMetrics,
) -> Result<MassSolveOutcome, ScenarioError> {
    let ds = scenario.deflection()?;
    required_impactor_mass(
        &ds,
        Epoch::from_tdb_seconds_past_j2000(arrival_tdb),
        metrics,
        IMPACTOR_BETA,
        threat_mass_kg(),
        SAFE_PERIGEE_TARGET_M,
        mass_solve_seed_kg(),
        mass_solve_cap_kg(),
        MASS_SOLVE_TOL,
    )
    .map_err(|e| ScenarioError::Integration(format!("required impactor mass: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whether the binding's kernel-gated tests can run. Goes through the shared
    /// resolver rather than reading the environment, so a developer who *has* the
    /// kernels but has not exported the variables runs the real suite instead of
    /// a green shell of it — and `ASTEROID_REQUIRE_KERNELS` makes the skip loud.
    fn have_kernels() -> bool {
        asteroid_core::kernels::resolve_for_test("the MissionCore kernel-gated tests").is_some()
    }

    /// Metres per AU — for authoring synthetic-body semi-major axes in SI.
    const AU_M: f64 = AU_KM * M_PER_KM;

    // --- The porkchop layer -------------------------------------------------

    /// **The frontend must not invent a third rock.** `THREAT_RADIUS_M` /
    /// `THREAT_DENSITY_KG_M3` exist only because `SrpParams::sub_km_rock` keeps the
    /// same two numbers as function locals, so the delivery layer had to restate
    /// them to get a mass. Restating is the hazard: nothing but this test stops the
    /// SRP toggle from modelling a 300 m body while the porkchop divides its Δv by
    /// the mass of some other one.
    ///
    /// Asserted through the *derived* quantity (`A/m = 3/(4rρ)`) rather than by
    /// comparing literals, so it fails for either side drifting — and note the core
    /// test suite already had a `2.0e10 kg` "~sub-km rock" in `mission.rs`, which is
    /// a different rock again (that one is a test fixture and stays one; this is
    /// what the shipping display divides by).
    #[test]
    fn threat_body_matches_the_srp_default() {
        let derived = 3.0 / (4.0 * THREAT_RADIUS_M * THREAT_DENSITY_KG_M3);
        let srp = SrpParams::sub_km_rock().area_to_mass_m2_per_kg;
        assert!(
            (derived - srp).abs() / srp < 1e-12,
            "the porkchop's threat body (r = {THREAT_RADIUS_M} m, ρ = {THREAT_DENSITY_KG_M3} \
             kg/m³ → A/m = {derived:.6e}) has drifted from the one SrpParams::sub_km_rock \
             models (A/m = {srp:.6e}). Two parts of the shipping model would be flying \
             different asteroids."
        );
        // And the mass those two imply, so a units slip in threat_mass_kg shows.
        let m = threat_mass_kg();
        assert!(
            (2.7e10..2.9e10).contains(&m),
            "a 300 m stony body should be ~2.83e10 kg, got {m:.4e}"
        );
    }

    /// The three blanks a porkchop must keep apart, on a real grid.
    ///
    /// A cell can be empty for three different reasons, and collapsing any pair
    /// erases something the operator needs: **no transfer exists at all** (`c3 =
    /// -1`), **a transfer exists but this launcher cannot reach its `C3`**
    /// (`payload = 0` with `c3 ≥ 0`), and **a transfer this launcher reaches but
    /// which projects poorly onto the track** (`payload > 0`, tiny `|Δv|`). The
    /// third is the module's whole thesis — deliverable ≠ well-aimed — and it is
    /// invisible if the first two are drawn the same way.
    ///
    /// Also pins the sentinel discipline the display depends on: **no `NaN` reaches
    /// the packed arrays**, because a single `NaN` poisons every min/max the
    /// heatmap normalizes by, turning the whole grid one flat colour.
    #[test]
    fn the_porkchop_grid_separates_its_three_blanks() {
        if !have_kernels() {
            return;
        }
        let mut mc = MissionCore::load().expect("kernels load");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");
        let scenario = mc.scenario_arc().expect("a built scenario");

        // Small axes: this test is about cell *semantics*, not resolution.
        let view = PorkchopView::build(&scenario, 12, 12).expect("grid builds");
        assert_eq!(view.launch_count(), 12);
        assert_eq!(view.arrival_count(), 12);

        let c3 = view.c3_flat();
        let revs = view.revolutions_flat();
        let along = view.along_track_flat();
        assert_eq!(c3.len(), 144);
        assert_eq!(revs.len(), 144);

        // No NaN anywhere, in any column, ever.
        for (name, col) in [
            ("c3", &c3),
            ("along_track", &along),
            ("arrival_v_rel", &view.arrival_v_rel_flat()),
        ] {
            assert!(
                col.iter().all(|v| v.is_finite()),
                "{name} carries a non-finite value — one NaN flattens the whole heatmap"
            );
        }

        // Blank cells are marked in c3 and *agree* with the revolutions column:
        // the two must never disagree about which cells hold a transfer.
        let blanks = c3.iter().filter(|v| **v < 0.0).count();
        let filled = 144 - blanks;
        assert!(
            filled > 0,
            "every cell came back NoTransfer — the axes are outside the propagated span"
        );
        for k in 0..144 {
            assert_eq!(
                c3[k] < 0.0,
                revs[k] < 0,
                "cell {k}: c3 and revolutions disagree about whether a transfer exists"
            );
        }

        // **Switching launcher must change something, or the vehicle-independent
        // grid bought nothing.** What it changes is *mass*, not *reach*: four of the
        // five tables stop at C3 ≈ 100 km²/s², so the launchers open very nearly the
        // same set of windows and differ ~3× in what they can put through one. That
        // is a real property of the LSP data and worth pinning, because the obvious
        // assertion — "the stronger rocket reaches more cells" — is trivially true
        // here (10 vs 10) and would have passed over a broken C3→mass map.
        let weak = launch_vehicle(0).expect("a first vehicle");
        let strong = launch_vehicle(launch_vehicle_count() - 1).expect("a last vehicle");
        let (pw, ps) = (view.payload_kg_flat(weak), view.payload_kg_flat(strong));
        let reach = |p: &[f64]| (0..144).filter(|&k| c3[k] >= 0.0 && p[k] > 0.0).count();
        println!(
            "porkchop {}×{}: {filled} transfers, {blanks} blank; {} reaches {}, {} reaches {}",
            view.launch_count(),
            view.arrival_count(),
            weak.name,
            reach(&pw),
            strong.name,
            reach(&ps)
        );
        let mut compared = 0;
        for k in 0..144 {
            if c3[k] >= 0.0 && pw[k] > 0.0 {
                assert!(
                    ps[k] > pw[k],
                    "cell {k} (C3 = {:.1}): {} delivers {:.0} kg but {} only {:.0} kg — \
                     the C3→payload mapping is inverted or reading one table twice",
                    c3[k],
                    strong.name,
                    ps[k],
                    weak.name,
                    pw[k]
                );
                compared += 1;
            }
        }
        assert!(
            compared > 0,
            "no cell was reachable by the weakest launcher, so nothing compared the two"
        );

        // A launcher's blanks must be a *subset* of "reachable" — a cell with no
        // transfer can never have a payload, or the display would offer a mission
        // through a window with no trajectory.
        let p = view.payload_kg_flat(strong);
        for k in 0..144 {
            if c3[k] < 0.0 {
                assert_eq!(
                    p[k], 0.0,
                    "cell {k} has no transfer yet reports {} kg deliverable",
                    p[k]
                );
            }
        }

        // The detail marshalling agrees with the columns, cell by cell — the
        // readout and the heatmap must never describe different cells.
        let k = (0..144)
            .find(|&k| c3[k] >= 0.0 && p[k] > 0.0)
            .expect("a feasible cell for the strongest launcher");
        let (i, j) = (k / 12, k % 12);
        let d = view.detail(i, j, strong).expect("detail for a filled cell");
        assert_eq!(d.c3_km2_s2, c3[k]);
        assert_eq!(d.revolutions as i32, revs[k]);
        assert_eq!(d.along_track_proj_ms, along[k]);
        assert!(
            d.tof_days >= MIN_TOF_DAYS - 1e-6,
            "cell {k} has a {:.1} d transfer, below the grid's own {MIN_TOF_DAYS} d floor",
            d.tof_days
        );
        // Δv is the delivered mass acting through the projection — including its
        // sign, which says which way the push moves the semi-major axis.
        let expect_dv = IMPACTOR_BETA * (d.payload_kg / threat_mass_kg()) * d.along_track_proj_ms;
        assert!((d.along_track_dv_ms - expect_dv).abs() <= 1e-12 * expect_dv.abs().max(1e-12));
        assert_eq!(
            d.along_track_dv_ms < 0.0,
            d.along_track_proj_ms < 0.0,
            "the delivered Δv lost the projection's sign — a retrograde push is a real \
             lever, not bad aim, and the readout must say which way it acts"
        );

        // And a blank cell has no detail at all, rather than a zero-filled row.
        if let Some(kb) = (0..144).find(|&k| c3[k] < 0.0) {
            assert!(
                view.detail(kb / 12, kb % 12, strong).is_none(),
                "a NoTransfer cell handed back a detail row"
            );
        }
        assert!(view.detail(99, 0, strong).is_none(), "out-of-range detail");
    }

    /// **How many laps the shipping grid allows, decided on numbers.**
    ///
    /// `DEFAULT_MAX_REVOLUTIONS` is the one free parameter of this view that
    /// changes what an operator *sees*: every extra lap is another family of
    /// cheaper transfers, so it moves cells from "no launcher can do this" to
    /// "here is a mission". The direct-only grid is known to be wrong (the 2.6 yr
    /// case: C3 933 direct vs 55 lapping), but "more is better" is not a reason to
    /// pick a number — each lap costs a ~2× step in grid time, and if the second
    /// lap opened nothing it would be pure cost.
    ///
    /// So this prints the reachable-window count and the wall-clock at `N = 0…3`
    /// and asserts only what it actually measures: that laps never *remove*
    /// windows (selection is a minimum over a growing candidate set, so a
    /// regression here means the selector is returning a worse option), and that
    /// the shipped default opens strictly more than the direct-only grid.
    #[test]
    fn revolutions_open_windows_and_what_they_cost() {
        if !have_kernels() {
            return;
        }
        let mut mc = MissionCore::load().expect("kernels load");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");
        let scenario = mc.scenario_arc().expect("a built scenario");
        let strong = launch_vehicle(launch_vehicle_count() - 1).expect("a last vehicle");

        let (n, cells) = (24usize, 24 * 24);
        let mut reachable = Vec::new();
        for max_rev in 0..=3u32 {
            let t0 = std::time::Instant::now();
            let view = PorkchopView::build_with_revolutions(&scenario, n, n, max_rev)
                .expect("grid builds");
            let dt = t0.elapsed();
            let c3 = view.c3_flat();
            let pay = view.payload_kg_flat(strong);
            let transfers = c3.iter().filter(|v| **v >= 0.0).count();
            let reach = (0..cells).filter(|&k| c3[k] >= 0.0 && pay[k] > 0.0).count();
            let lapping = view.revolutions_flat().iter().filter(|r| **r >= 1).count();
            let cheapest = c3
                .iter()
                .filter(|v| **v >= 0.0)
                .fold(f64::INFINITY, |a, b| a.min(*b));
            println!(
                "N<={max_rev}: {transfers:3}/{cells} transfers, {lapping:3} lapping, \
                 {reach:3} reachable by {}, cheapest C3 {cheapest:7.2} km²/s², \
                 {:.0} ms ({:.1} µs/cell)",
                strong.name,
                dt.as_secs_f64() * 1e3,
                dt.as_secs_f64() * 1e6 / cells as f64
            );
            reachable.push(reach);
        }

        // **Laps take the grid below where the vehicle tables start**, and that is
        // the condition making the flat-hold in `payload_kg` load-bearing rather
        // than theoretical. The cheapest cell drops to C3 ≈ 0.34 km²/s² once one
        // lap is allowed, under the 1.0 that three of the five tables begin at —
        // and `payload_kg` used to fail closed there, so the heatmap drew those
        // easily-flyable windows as unreachable and captioned them *too much C3*.
        //
        // Checked here rather than on the 12×12 grid of the test above, where it
        // would be **vacuous**: that grid's cheapest cell is C3 3.64, above every
        // floor, so the assertion would pass without ever entering the regime it
        // exists to protect. Asserting the regime is reached comes first, for
        // exactly that reason.
        let deep = PorkchopView::build_with_revolutions(&scenario, n, n, DEFAULT_MAX_REVOLUTIONS)
            .expect("grid builds");
        let deep_c3 = deep.c3_flat();
        let cheapest = deep_c3
            .iter()
            .filter(|v| **v >= 0.0)
            .fold(f64::INFINITY, |a, b| a.min(*b));
        let floors: Vec<f64> = LAUNCH_VEHICLES.iter().map(|v| v.min_c3_km2_s2()).collect();
        println!("cheapest cell C3 = {cheapest:.3} km²/s²; vehicle table floors: {floors:?}");
        assert!(
            floors.iter().any(|f| cheapest < *f),
            "the grid's cheapest cell (C3 {cheapest:.3}) is above every vehicle's table \
             floor {floors:?}, so the below-the-table behaviour is not being exercised \
             at all — this check has gone vacuous"
        );
        for v in LAUNCH_VEHICLES {
            assert!(
                v.payload_kg(cheapest) > 0.0,
                "{} delivers nothing at the grid's cheapest cell (C3 = {cheapest:.3}), \
                 below its table floor of {:.2} — a cheaper departure is an *easier* \
                 one, and zeroing it draws a real window as unreachable",
                v.name,
                v.min_c3_km2_s2()
            );
        }

        for w in reachable.windows(2) {
            assert!(
                w[1] >= w[0],
                "allowing another lap REMOVED reachable windows ({} → {}) — the \
                 selection is a minimum over a growing candidate set, so it can only \
                 ever get cheaper",
                w[0],
                w[1]
            );
        }
        let shipped = reachable[DEFAULT_MAX_REVOLUTIONS as usize];
        assert!(
            shipped > reachable[0],
            "the shipped lap budget ({DEFAULT_MAX_REVOLUTIONS}) opens {shipped} windows, \
             no more than the direct-only grid's {} — it is paying a ~{}× per-cell cost \
             for nothing",
            reachable[0],
            DEFAULT_MAX_REVOLUTIONS + 1
        );
    }

    /// **The on-demand verify is the honest half**, and this pins the two things it
    /// could get silently wrong: that it is really re-flying the field (zero
    /// delivered mass must reproduce the *nominal hit*, which catches a wrong epoch,
    /// frame, or un-applied impulse), and that a real delivery moves the b-plane.
    ///
    /// The verdict is read as **`|B|` against `b_capture`** — the pair the core's own
    /// `is_hit` compares — never `perigee` against `capture_radius`, which is neither
    /// coherent pair and is ~1.5× too strict (the 3C-2c bug this project already
    /// shipped once).
    #[test]
    fn verifying_a_cell_reproduces_the_nominal_then_moves_it() {
        if !have_kernels() {
            return;
        }
        let mut mc = MissionCore::load().expect("kernels load");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");
        let nominal_b = mc
            .nominal_impact_parameter_m()
            .expect("a nominal encounter");
        let capture = mc.capture_radius_m().expect("a capture radius");
        assert!(nominal_b <= capture, "the nominal must be a hit");
        let scenario = mc.scenario_arc().expect("a built scenario");

        let view = PorkchopView::build(&scenario, 10, 10).expect("grid builds");
        let c3 = view.c3_flat();
        let along = view.along_track_flat();
        // The best-coupled feasible cell: the strongest launcher, the largest
        // |along-track projection| — the window most likely to actually work.
        let strong = launch_vehicle(launch_vehicle_count() - 1).expect("a last vehicle");
        let pay = view.payload_kg_flat(strong);
        let k = (0..c3.len())
            .filter(|&k| c3[k] >= 0.0 && pay[k] > 0.0)
            .max_by(|&a, &b| along[a].abs().total_cmp(&along[b].abs()))
            .expect("a feasible cell");
        let (i, j) = (k / 10, k % 10);
        let d = view.detail(i, j, strong).expect("detail");
        let metrics = view.metrics_at(i, j).expect("metrics");
        println!(
            "verify cell ({i},{j}): C3 {:.1} km²/s², N={}, TOF {:.0} d, {} delivers {:.0} kg \
             → along-track {:+.3} m/s",
            d.c3_km2_s2, d.revolutions, d.tof_days, strong.name, d.payload_kg, d.along_track_dv_ms
        );

        // (1) Zero delivered mass ⇒ zero impulse ⇒ the nominal hit, to the metre.
        match verify_porkchop_cell(&scenario, d.arrival_tdb, &metrics, 0.0).expect("verify runs") {
            CellVerdict::Encounter {
                impact_parameter_m,
                capture_radius_m,
                is_hit,
                ..
            } => {
                assert!(
                    (impact_parameter_m - nominal_b).abs() / nominal_b < 1e-3,
                    "a zero-mass verify gave |B| = {impact_parameter_m:.1} m but the nominal \
                     is {nominal_b:.1} m — the impulse path is reading the wrong epoch/frame"
                );
                assert!((capture_radius_m - capture).abs() / capture < 1e-6);
                assert!(is_hit, "zero deflection must still be the hit");
            }
            other => panic!("zero mass should reproduce the nominal encounter, got {other:?}"),
        }

        // (2) A real delivery moves it. The launcher's *own* deliverable mass may or
        // may not be enough — that is the honest answer and not something to assert —
        // so the discriminating claim is that the b-plane MOVED, and moved outward
        // for a prograde push. A verify that silently ignored its mass would fail
        // here while passing (1).
        let verdict =
            verify_porkchop_cell(&scenario, d.arrival_tdb, &metrics, d.payload_kg).expect("verify");
        let moved_b = match verdict {
            CellVerdict::CleanMiss => f64::INFINITY,
            CellVerdict::Encounter {
                impact_parameter_m, ..
            } => impact_parameter_m,
            CellVerdict::NotHyperbolic => 0.0,
        };
        println!(
            "  {} kg through this window: |B| {nominal_b:.0} → {moved_b:.0} m (capture {capture:.0} m)",
            d.payload_kg as i64
        );
        assert!(
            (moved_b - nominal_b).abs() > 1.0,
            "delivering {:.0} kg changed |B| by less than a metre ({nominal_b:.1} → \
             {moved_b:.1}) — the impactor mass is not reaching the propagation",
            d.payload_kg
        );

        // (3) A deliberately huge impactor must clear the capture disc — the
        // verdict path itself works, judged on the coherent pair.
        let huge = 50.0 * threat_mass_kg() / (IMPACTOR_BETA * metrics.arrival_v_rel_ms);
        let big =
            verify_porkchop_cell(&scenario, d.arrival_tdb, &metrics, huge).expect("verify runs");
        match big {
            CellVerdict::CleanMiss => {}
            CellVerdict::Encounter {
                impact_parameter_m,
                capture_radius_m,
                is_hit,
                ..
            } => {
                assert!(
                    !is_hit && impact_parameter_m > capture_radius_m,
                    "a {huge:.2e} kg impactor left |B| = {impact_parameter_m:.0} m inside \
                     b_capture = {capture_radius_m:.0} m"
                );
            }
            CellVerdict::NotHyperbolic => {
                panic!("a huge deflection should not be a dead-centre hit")
            }
        }
    }

    /// **The required-mass solve, judged on what it delivers rather than on what it
    /// returns.** Two cells of one grid, and the two outcomes are both real answers.
    ///
    /// The discriminating claim is the round trip: take the mass the solver says
    /// this window needs, fly *that* mass through the independent verify path, and
    /// require the perigee it reaches to actually clear the target. A test that
    /// asserted only `impactor_mass_kg > 0` would pass over a mis-bracketed solve, a
    /// wrong target, or a solver reading a different cell's geometry — this is the
    /// same catch the core suite makes kernel-free, made here on the real field
    /// through the exact call the frontend makes.
    ///
    /// The second cell pins the guard: a window whose arrival barely projects onto
    /// the track is not deflected by any mass worth launching, and must come back
    /// [`MassSolveOutcome::InfeasibleAtCap`] — an honest window state that the
    /// frontend renders as data. Both outcomes from one grid, so neither is a
    /// specially-built fixture.
    ///
    /// Prints its own cost, because that cost is the whole reason the constants
    /// above are what they are and a 5× regression should be visible rather than
    /// merely slow.
    #[test]
    fn required_mass_is_what_the_window_actually_needs() {
        if !have_kernels() {
            return;
        }
        use std::time::Instant;

        let mut mc = MissionCore::load().expect("kernels load");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");
        let scenario = mc.scenario_arc().expect("a built scenario");

        let n = 10;
        let view = PorkchopView::build(&scenario, n, n).expect("grid builds");
        let c3 = view.c3_flat();
        let along = view.along_track_flat();
        let transfers: Vec<usize> = (0..c3.len()).filter(|&k| c3[k] >= 0.0).collect();
        assert!(
            !transfers.is_empty(),
            "no transfers in the grid — nothing to solve against"
        );

        // Best-coupled window: the one an operator would actually pick, and the one
        // most likely to be reachable at all.
        let best = *transfers
            .iter()
            .max_by(|&&a, &&b| along[a].abs().total_cmp(&along[b].abs()))
            .expect("a cell");
        // Worst-coupled window, restricted to the **late half of the arrival axis**.
        // Not to bias the outcome — a late, badly-projected window is infeasible for
        // both reasons at once — but for cost: a probe re-flies from arrival to the
        // encounter, so a late arrival is ~3× cheaper per probe (5.8 s vs 18.2 s
        // measured) and this cell walks the full doubling ladder to the cap.
        let weak = *transfers
            .iter()
            .filter(|&&k| k % n >= n / 2)
            .min_by(|&&a, &&b| along[a].abs().total_cmp(&along[b].abs()))
            .expect("a late cell");

        let target = SAFE_PERIGEE_TARGET_M;
        let cap = mass_solve_cap_kg();
        println!(
            "target perigee {:.0} km, seed {:.0} kg, cap {cap:.0} kg (= 100 x the best launch)",
            target / 1000.0,
            mass_solve_seed_kg()
        );

        // --- (1) The best-coupled window: a number, and it must be the right one.
        let (bi, bj) = (best / n, best % n);
        let b_metrics = view.metrics_at(bi, bj).expect("metrics");
        let b_arrival = view.arrival_tdb()[bj];
        let t0 = Instant::now();
        let out = required_cell_mass(&scenario, b_arrival, &b_metrics).expect("mass solve");
        let solve_secs = t0.elapsed().as_secs_f64();
        println!(
            "best-coupled cell ({bi},{bj}): |along| {:.0} m/s, arrival lead {:.2} yr \
             → {out:?} in {solve_secs:.1} s",
            along[best].abs(),
            (mc.impact_tdb_seconds() - b_arrival) / 3.155_76e7
        );
        let MassSolveOutcome::Feasible { impactor_mass_kg } = out else {
            panic!(
                "the best-coupled window in the grid came back over the {cap:.0} kg cap \
                 ({out:?}) — either the cap is sized wrong or the coupling is not reaching \
                 the propagation; the view would then never show a mass at all"
            )
        };
        assert!(
            impactor_mass_kg > 0.0 && impactor_mass_kg <= cap,
            "solved mass {impactor_mass_kg} kg is outside the bracket (0, {cap}]"
        );

        // **The round trip.** Fly the solved mass through the verify path — a
        // different function, the one `[E]` uses — and require the perigee it reaches
        // to clear the target. This is what "required" has to mean.
        let reached = match verify_porkchop_cell(&scenario, b_arrival, &b_metrics, impactor_mass_kg)
            .expect("verify runs")
        {
            // Off the scan gate entirely: past the target by an unmeasured margin.
            CellVerdict::CleanMiss => f64::INFINITY,
            CellVerdict::Encounter { perigee_m, .. } => perigee_m,
            CellVerdict::NotHyperbolic => 0.0,
        };
        println!(
            "  {impactor_mass_kg:.0} kg through this window reaches perigee {:.0} km \
             (target {:.0} km)",
            reached / 1000.0,
            target / 1000.0
        );
        assert!(
            reached >= target,
            "the solver called {impactor_mass_kg:.0} kg sufficient, but flying it reaches \
             perigee {reached:.0} m against a {target:.0} m target — the bracket is on the \
             wrong side of the crossing"
        );

        // …and it must be *minimal*, not merely sufficient: a mass a comfortable
        // margin below the answer has to FAIL the same target. Without this, returning
        // the cap (or any large number) would pass everything above.
        let under = impactor_mass_kg * (1.0 - 4.0 * MASS_SOLVE_TOL);
        let under_reached =
            match verify_porkchop_cell(&scenario, b_arrival, &b_metrics, under).expect("verify") {
                CellVerdict::CleanMiss => f64::INFINITY,
                CellVerdict::Encounter { perigee_m, .. } => perigee_m,
                CellVerdict::NotHyperbolic => 0.0,
            };
        println!(
            "  {under:.0} kg (−{:.0}%) reaches {:.0} km — short, as a minimum requires",
            400.0 * MASS_SOLVE_TOL,
            under_reached / 1000.0
        );
        assert!(
            under_reached < target,
            "{under:.0} kg already reaches perigee {under_reached:.0} m ≥ the {target:.0} m \
             target, so {impactor_mass_kg:.0} kg is not the requirement — it is an upper bound \
             being reported as one"
        );

        // --- (2) The worst-coupled window: the honest "no", not a hang or an error.
        let (wi, wj) = (weak / n, weak % n);
        let w_metrics = view.metrics_at(wi, wj).expect("metrics");
        let w_arrival = view.arrival_tdb()[wj];
        let t1 = Instant::now();
        let w_out = required_cell_mass(&scenario, w_arrival, &w_metrics).expect("mass solve");
        println!(
            "worst-coupled late cell ({wi},{wj}): |along| {:.0} m/s → {w_out:?} in {:.1} s",
            along[weak].abs(),
            t1.elapsed().as_secs_f64()
        );
        match w_out {
            MassSolveOutcome::InfeasibleAtCap {
                mass_cap_kg,
                perigee_reached_m,
            } => {
                assert_eq!(mass_cap_kg, cap);
                assert!(
                    perigee_reached_m < target,
                    "InfeasibleAtCap reported perigee {perigee_reached_m:.0} m, which already \
                     clears the {target:.0} m target — then it was not infeasible"
                );
            }
            // Not a failure of the code — a genuinely better grid than the one this
            // was written against. Say so loudly rather than assert a stale fact.
            MassSolveOutcome::Feasible { impactor_mass_kg } => panic!(
                "even the worst-coupled late window is feasible at {impactor_mass_kg:.0} kg. \
                 That is a real result, not a bug, but it means this grid no longer exercises \
                 the InfeasibleAtCap path the frontend renders — pick a harder cell or lower \
                 the cap."
            ),
        }
    }

    /// 2035-01-01 TDB — comfortably inside the de440s span; the synthetic-body
    /// seed epoch for the catalog tests.
    fn epoch_2035() -> Epoch {
        Epoch::from_tdb_gregorian(2035, 1, 1, 0, 0, 0, 0)
    }

    /// The small-body mount, end to end: an unmounted core must **refuse** Ceres,
    /// and a mounted one must place it in the main belt.
    ///
    /// The refusal half is the half that matters. `body_position_ecl_au` returning
    /// `None` is what the display gates on; if an unmounted almanac ever answered
    /// with something, that something would be drawn, and a body drawn at a bad
    /// position in a heliocentric view is not a glitch — `Vector3::ZERO` *is* the
    /// Sun. This project has shipped that confusion three times.
    ///
    /// The display scenery list [`SB441_BODIES`] and the core's canonical
    /// **force-perturber** table (`asteroid_core::SB441_PERTURBER_GM_AU3_DAY2`) are
    /// two hand-written spellings of the same sixteen bodies — one for drawing, one
    /// for gravity. They must never drift: a body the map labels that the force
    /// field omits (or vice-versa) is exactly the kind of silent inconsistency this
    /// project keeps paying for. This pins them together by `(id, name)`, kernel-free,
    /// so an edit to either list that desyncs the two fails at `cargo test` rather
    /// than shipping a scene whose scenery and physics disagree about the belt.
    #[test]
    fn scenery_and_force_perturber_lists_agree() {
        let force = asteroid_core::SB441_PERTURBER_GM_AU3_DAY2;
        assert_eq!(
            SB441_BODIES.len(),
            force.len(),
            "the scenery and force-perturber lists have different lengths"
        );
        for (scenery, (fid, fname, _gm)) in SB441_BODIES.iter().zip(force.iter()) {
            assert_eq!(
                (scenery.0, scenery.1),
                (*fid, *fname),
                "scenery body {scenery:?} does not match force perturber ({fid}, {fname}) \
                 — the two sb441 lists have drifted"
            );
        }
    }

    /// The positive half also pins the ids: `SB441_BODIES` was read out of the
    /// kernel's segment table, and a wrong id would resolve to nothing (or, worse,
    /// to some other body) rather than announce itself.
    #[test]
    fn small_body_mount_gates_and_resolves() {
        if !have_kernels() {
            return;
        }
        let Some(k) = asteroid_core::kernels::resolve() else {
            return;
        };
        let Some(sb) = k.small_bodies.clone() else {
            // No 646 MB kernel on this machine — the optional half of the contract.
            // Failing here would punish a valid setup.
            //
            // But note what this skip is NOT covered by: `ASTEROID_REQUIRE_KERNELS`
            // deliberately does not catch it, because sb441 is genuinely optional in
            // a way the DE pair is not. So on a box with the pair and not the
            // small-body file, this test prints green having asserted nothing about
            // mounting — the exact silent-green shape that cost this project two
            // verification claims in July. The `eprintln` below is swallowed for a
            // passing test; the wall clock is again the only tell (~0.3 s warm here
            // versus instant). If this becomes load-bearing in CI, it wants its own
            // require-flag rather than a stricter reading of the existing one.
            eprintln!("no small-body kernel resolved — skipping the mount test");
            return;
        };

        let (bsp, pca) = k.as_strs();
        let t = epoch_2035().tdb_seconds_past_j2000();

        // Unmounted: armed or not, an asteroid is not reachable until the mount.
        let mut core = MissionCore::load_from(bsp, pca).expect("load");
        assert!(!core.small_bodies_mounted(), "nothing mounted yet");
        assert!(
            core.body_position_ecl_au(2000001, t).is_none(),
            "an unmounted almanac answered for Ceres — that answer becomes a body \
             drawn on the Sun"
        );
        // Arming records a path and mounts nothing, so the refusal must survive it.
        core.set_small_body_kernel(sb.to_str().unwrap())
            .expect("arm");
        assert!(
            !core.small_bodies_mounted() && core.body_position_ecl_au(2000001, t).is_none(),
            "arming a kernel is not mounting it"
        );

        // Mounted: every id in the table resolves, and lands in the main belt.
        let eph = mount_small_bodies(&k.bsp, &k.pca, &sb).expect("mount");
        for (id, name) in SB441_BODIES {
            let p = eph
                .position_km(
                    Frame::from_ephem_j2000(*id),
                    SUN_J2000,
                    Epoch::from_tdb_seconds_past_j2000(t).as_hifitime(),
                )
                .unwrap_or_else(|e| panic!("{name} ({id}) did not resolve: {e}"));
            let r_au = p.norm() / AU_KM;
            assert!(
                (1.5..5.5).contains(&r_au),
                "{name} ({id}) is {r_au:.3} AU from the Sun — not a main-belt \
                 distance, so this id is not the body it claims to be"
            );
        }
    }

    /// The obliquity rotation is a pure rotation: it preserves length and leaves
    /// a vector in the equatorial plane (z_eq = 0) with its z-component still
    /// zero only along the shared X axis. Concretely, the ecliptic north pole
    /// `(0,0,1)` AU·AU_KM in ecliptic came from ICRF `(0,−sinε,cosε)`. Kernel-free.
    #[test]
    fn obliquity_rotation_is_orthonormal_about_x() {
        // A point on the ICRF x-axis is unchanged in y/z.
        let on_x = icrf_km_to_ecliptic_au(Vector3::new(AU_KM, 0.0, 0.0));
        assert!((on_x.x - 1.0).abs() < 1e-12);
        assert!(on_x.y.abs() < 1e-12 && on_x.z.abs() < 1e-12);

        // Length preserved (rotation), checked on an oblique vector.
        let v = Vector3::new(0.3 * AU_KM, -0.7 * AU_KM, 0.5 * AU_KM);
        let r = icrf_km_to_ecliptic_au(v);
        assert!((r.norm() - v.norm() / AU_KM).abs() < 1e-12);

        // The ICRF celestial pole (0,0,1) tilts to ecliptic latitude 90°−ε: its
        // ecliptic y is +sinε, z is +cosε (pole leans toward +Y in ecliptic).
        let pole = icrf_km_to_ecliptic_au(Vector3::new(0.0, 0.0, AU_KM));
        let eps = OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0;
        assert!((pole.x).abs() < 1e-12);
        assert!((pole.y - eps.sin()).abs() < 1e-12);
        assert!((pole.z - eps.cos()).abs() < 1e-12);
    }

    /// The b-plane display basis, kernel-free. Two things worth pinning here.
    ///
    /// **The reference pole is an ICRF vector.** The b-plane frame is built from
    /// `Ŝ` and the ecliptic north pole, and everything it touches (`Ŝ`, `B`, the
    /// geocentric tracks) is ICRF — so the pole must be ICRF too. The obvious
    /// `(0, 0, 1)` is the pole in *ecliptic* coordinates and is wrong here by the
    /// 23.4° obliquity. This asserts the relationship that makes it right:
    /// `ecliptic_north_icrf()` is exactly the vector `icrf_km_to_ecliptic_au` maps
    /// onto ecliptic `(0, 0, 1)`. Get this wrong and nothing errors — the plot just
    /// quietly tilts.
    ///
    /// **The frame is orthonormal and never NaNs**, including for a `Ŝ` parallel to
    /// the pole, where `Ŝ × N̂` vanishes and the recipe has nothing to work with. A
    /// normalise of that zero would produce a NaN basis and an invisible plot, so
    /// the fallback is exercised rather than assumed.
    #[test]
    fn bplane_basis_is_orthonormal_and_references_the_pole_in_icrf() {
        // The pole used here must be the ICRF vector that IS ecliptic north.
        let north_ecl = icrf_km_to_ecliptic_au(ecliptic_north_icrf() * AU_KM);
        assert!(
            (north_ecl - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12,
            "ecliptic_north_icrf() must rotate to ecliptic (0,0,1), got {north_ecl:?} — \
             the b-plane frame would be tilted by the obliquity"
        );
        // …and it is emphatically not (0,0,1) itself: that is the trap.
        assert!(
            (ecliptic_north_icrf() - Vector3::new(0.0, 0.0, 1.0)).norm() > 0.3,
            "the ICRF and ecliptic poles must differ by the obliquity (~23.4°)"
        );

        let check_orthonormal = |s: Vector3<f64>, label: &str| {
            let (xi, zeta, s_out) = bplane_basis(s).unwrap_or_else(|| panic!("{label}: no basis"));
            for (v, n) in [(xi, "ξ̂"), (zeta, "ζ̂"), (s_out, "Ŝ")] {
                assert!(
                    (v.norm() - 1.0).abs() < 1e-12,
                    "{label}: {n} is not unit ({})",
                    v.norm()
                );
                assert!(
                    v.iter().all(|c| c.is_finite()),
                    "{label}: {n} is not finite"
                );
            }
            assert!(xi.dot(&zeta).abs() < 1e-12, "{label}: ξ̂·ζ̂ ≠ 0");
            assert!(xi.dot(&s_out).abs() < 1e-12, "{label}: ξ̂·Ŝ ≠ 0");
            assert!(zeta.dot(&s_out).abs() < 1e-12, "{label}: ζ̂·Ŝ ≠ 0");
        };

        // A generic asymptote, out of every coordinate plane.
        check_orthonormal(Vector3::new(0.36, -0.48, 0.8).normalize(), "generic");
        // The degenerate case the fallback exists for: straight down the pole.
        check_orthonormal(ecliptic_north_icrf(), "along the ecliptic pole");
        check_orthonormal(-ecliptic_north_icrf(), "against the ecliptic pole");
        // And the ICRF axes, for good measure.
        check_orthonormal(Vector3::x(), "ICRF +x");
        check_orthonormal(Vector3::z(), "ICRF +z");

        // In the non-degenerate case ξ̂ really is perpendicular to the pole (it is
        // Ŝ × N̂), which is what makes ζ̂ the "roughly south" axis the plot draws down.
        let (xi, _, _) = bplane_basis(Vector3::new(0.36, -0.48, 0.8).normalize()).unwrap();
        assert!(
            xi.dot(&ecliptic_north_icrf()).abs() < 1e-12,
            "ξ̂ must lie in the ecliptic plane (ξ̂ = Ŝ × N̂ ⇒ ξ̂·N̂ = 0)"
        );

        // Garbage in, `None` out — never a NaN basis across the FFI.
        assert!(bplane_basis(Vector3::zeros()).is_none());
        assert!(bplane_basis(Vector3::new(f64::NAN, 0.0, 0.0)).is_none());
        assert!(bplane_basis(Vector3::new(f64::INFINITY, 0.0, 1.0)).is_none());
    }

    /// Kernel-gated. Every NAIF id the orrery display draws must resolve at
    /// **both edges** of the usable span, not just mid-span — a failed lookup
    /// returns `None`, which the binding maps to `Vector3::ZERO`, and ZERO is the
    /// *Sun's* position in this heliocentric frame. So a body that falls out of
    /// coverage does not render as visibly broken; it renders silently sitting on
    /// the Sun. This test is what stands between that and a shipped display.
    ///
    /// Two id choices are pinned here because the obvious guess is wrong:
    /// **Earth is 399, never 3** (3 is the Earth–Moon barycenter — the ~4671 km
    /// footgun of HANDOFF §5), and **Mars is 4, not 499** (de440s carries no Mars
    /// *geocenter* segment at all; the barycenter is all there is, and it sits
    /// within a few km of the planet, so it is harmless here — unlike Earth's).
    #[test]
    fn display_naif_ids_resolve_across_the_whole_usable_span() {
        if !have_kernels() {
            eprintln!("skipping display_naif_ids_*: no DE kernel");
            return;
        }
        let mc = MissionCore::load().expect("load kernels");
        let (span_lo, span_hi) = mc.usable_span_tdb();

        // The exact id list the orrery draws, with the heliocentric distance band
        // each must land in (AU) anywhere in the span. Bands are wide enough for
        // the real eccentric excursion, tight enough to catch a wrong body.
        let bodies: [(i32, &str, f64, f64); 8] = [
            (199, "MERCURY", 0.30, 0.48),
            (299, "VENUS", 0.71, 0.74),
            (399, "EARTH", 0.98, 1.02),
            (4, "MARS", 1.38, 1.68),
            (5, "JUPITER", 4.94, 5.46),
            (6, "SATURN", 8.99, 10.10),
            (7, "URANUS", 18.28, 20.10),
            (8, "NEPTUNE", 29.79, 30.33),
        ];

        for t in [span_lo, 0.0, span_hi] {
            for (id, name, lo, hi) in bodies {
                let p = mc.body_position_ecl_au(id, t).unwrap_or_else(|| {
                    panic!(
                        "{name} (NAIF {id}) does not resolve at TDB {t:.0} — it would render \
                         silently at the Sun, not visibly missing"
                    )
                });
                assert!(
                    (lo..=hi).contains(&p.norm()),
                    "{name} (NAIF {id}) at TDB {t:.0}: {:.3} AU outside [{lo}, {hi}]",
                    p.norm()
                );
                assert_ne!(p, Vector3::zeros(), "{name} returned the Sun's position");
            }
        }

        // Mars has no geocenter segment in de440s — pinned so a future "tidy-up"
        // to 499 (matching Earth's 399) fails loudly here instead of silently at
        // the Sun. If a mounted kernel ever gains it, prefer it and update this.
        assert!(
            mc.body_position_ecl_au(499, 0.0).is_none(),
            "this kernel resolves Mars 499 — prefer the geocenter over the \
             barycenter in the display and update this test"
        );
    }

    /// Kernel-gated. The discovered span must be genuinely usable at both edges
    /// and genuinely exhausted just outside them — the property the frontend's
    /// clock clamp relies on. Asserts the *shape* (a sane multi-century window
    /// bracketing J2000), not hardcoded dates, since the mounted kernel decides
    /// them: de440s ≈ 1850–2149, de441 ≈ 1550–2650.
    #[test]
    fn discovered_span_is_usable_inside_and_exhausted_outside() {
        if !have_kernels() {
            eprintln!("skipping discovered_span_*: no DE kernel");
            return;
        }
        let mc = MissionCore::load().expect("load kernels");
        let (lo, hi) = mc.usable_span_tdb();
        let year = 365.25 * 86_400.0;

        assert!(lo < 0.0 && hi > 0.0, "span should bracket J2000");
        assert!(
            (hi - lo) / year > 100.0,
            "span {:.0} yr implausibly short for a DE kernel",
            (hi - lo) / year
        );
        // Inside at both edges…
        assert!(
            mc.body_position_ecl_au(399, lo).is_some(),
            "span lo unusable"
        );
        assert!(
            mc.body_position_ecl_au(399, hi).is_some(),
            "span hi unusable"
        );
        // …and exhausted a year out, so the span is the real edge, not a guess
        // that happens to be conservative by decades.
        assert!(
            mc.body_position_ecl_au(399, lo - year).is_none(),
            "a year below the span still resolves — discovery under-reports coverage"
        );
        assert!(
            mc.body_position_ecl_au(399, hi + year).is_none(),
            "a year above the span still resolves — discovery under-reports coverage"
        );
    }

    /// Kernel-gated (release-run for speed). Loads the real DE440 kernels and
    /// checks the body-position path against physics + a *direct* ephemeris call:
    /// Earth ~1 AU from the Sun and essentially in the ecliptic plane (|z| ≪ 1),
    /// which it would NOT be (|z| up to ~0.4 AU) if the obliquity rotation were
    /// dropped — so this pins the rotation end-to-end. Skips green offline.
    #[test]
    fn body_positions_match_direct_ephemeris_and_lie_in_ecliptic() {
        if !have_kernels() {
            eprintln!("skipping body_positions_*: no DE kernel");
            return;
        }
        let mc = MissionCore::load().expect("load kernels");
        // 2035-01-01 TDB, comfortably inside the de440s span and the campaign.
        let t = Epoch::from_tdb_gregorian(2035, 1, 1, 0, 0, 0, 0).tdb_seconds_past_j2000();

        let earth = mc.body_position_ecl_au(399, t).expect("earth position");
        assert!(
            (0.98..=1.02).contains(&earth.norm()),
            "Earth heliocentric distance {:.4} AU not ~1 AU",
            earth.norm()
        );
        assert!(
            earth.z.abs() < 0.02,
            "Earth ecliptic z {:.4} AU too large — obliquity rotation likely wrong/missing",
            earth.z
        );

        // Direct ephemeris call, rotated by the same helper, must match exactly.
        let direct = mc
            .ephemeris
            .position_km(
                Frame::from_ephem_j2000(399),
                SUN_J2000,
                Epoch::from_tdb_seconds_past_j2000(t).as_hifitime(),
            )
            .expect("direct earth position");
        let direct_ecl = icrf_km_to_ecliptic_au(direct);
        assert!(
            (earth - direct_ecl).norm() < 1e-12,
            "body_position_ecl_au disagrees with a direct ephemeris call"
        );

        // Jupiter (barycenter, NAIF 5) is ~5.2 AU — a second, well-separated body.
        let jup = mc.body_position_ecl_au(5, t).expect("jupiter position");
        assert!(
            (4.9..=5.5).contains(&jup.norm()),
            "Jupiter heliocentric distance {:.3} AU not ~5.2 AU",
            jup.norm()
        );
    }

    /// Kernel-gated (release-run). Builds the default scenario and checks the
    /// binding's `required_dv_along_track` reproduces the cached `curve.json`
    /// points for the same fixed config — proving the deflection path is wired
    /// right, not just that it runs. Values are the deterministic output of
    /// `ImpactorConfig::default()`; if that config changes, regenerate curve.json
    /// and update these. Skips green offline.
    #[test]
    fn required_dv_matches_curve_json() {
        if !have_kernels() {
            eprintln!("skipping required_dv_matches_curve_json: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        // The recorded `curve.json` numbers below were swept against **20 000 km**.
        // That value now has a name, and the two must not drift apart: if the campaign
        // target ever moves, every `expected` here is stale and so is the launch-window
        // map's required-mass number, which is quoted against the same bar precisely so
        // the two compose. Fail here rather than let a Δv requirement and a mass
        // requirement silently describe different missions.
        let target = SAFE_PERIGEE_TARGET_M;
        assert_eq!(
            target, 2.0e7,
            "the campaign safe-perigee target moved; curve.json (and the expectations \
             below) were swept against 20 000 km and must be regenerated"
        );
        // (lead_seconds, required_dv) pairs straight from curve.json.
        let cases = [
            (12_464_104.312150536_f64, 0.587_75_f64), // 0.5 period
            (24_928_208.624301072, 0.509_75),         // 1.0 period
            (49_856_417.248602144, 0.255_125),        // 2.0 periods
        ];
        for (lead, expected) in cases {
            let dv = mc.required_dv_along_track(lead, target).expect("dv solve");
            let rel = (dv - expected).abs() / expected;
            assert!(
                rel < 0.02,
                "lead {lead:.0}s: dv {dv:.5} vs curve.json {expected:.5} (rel {rel:.3})"
            );
        }
    }

    /// Kernel-gated (release-run). **The deflection spectrum, measured on *this*
    /// threat rather than on the paper's.**
    ///
    /// `core::deflection`'s own comparison test quotes the two methods against
    /// each other on UCRL-PROC-228569's published 1 km body, because core must not
    /// learn about this campaign's rock. That table is citable and it is *not*
    /// transferable: it concludes the nuclear option deflects intact, which is
    /// true of a 1.05e12 kg body and false here. This is the same trap the J2 pair
    /// already caught — a per-term row has to be measured on the seed it will be
    /// displayed against, not on whichever body the literature used.
    ///
    /// So this re-runs the comparison at one bar on the shipping threat:
    /// [`threat_mass_kg`] / [`THREAT_RADIUS_M`], the live full-field
    /// `required_dv_along_track` at each lead, and [`SAFE_PERIGEE_TARGET_M`] —
    /// the same target `curve.json` and the launch-window map's required-mass
    /// figure are quoted against.
    ///
    /// # The result, and why it is the interesting one
    /// The threat is a **300 m** body: 2.83e10 kg, surface escape speed **0.159
    /// m/s**. The required Δv runs from ~8.4 m/s at a tenth of an orbit to ~0.066
    /// m/s at eight orbits — which means **every lead time the campaign covers
    /// needs a Δv larger than the body's own escape speed**, or within a factor of
    /// three of it. A standoff burst sized to deliver that does not deflect this
    /// rock, it disperses it.
    ///
    /// That is not a failure of the term; it is the term reporting the thing
    /// LLNL-PROC-485160 says in words — *"[a]t a size of 100 meters ... inducing a
    /// 1 cm/s speed change will almost certainly result in extensive debris
    /// ejection or fragmentation. Fortunately, bodies of this size may be
    /// addressed by impactors."* §5 asks for the methods to be modelled as a
    /// **spectrum across lead time**; this asserts where on that spectrum the
    /// campaign's own body actually sits, instead of restating the spectrum.
    #[test]
    fn deflection_methods_compared_at_one_bar_on_the_real_threat() {
        if !have_kernels() {
            eprintln!(
                "skipping deflection_methods_compared_at_one_bar_on_the_real_threat: no DE kernel"
            );
            return;
        }
        use asteroid_core::deflection::{
            disruption_regime, escape_speed_ms, kinetic_impactor_mass_for_dv, DisruptionRegime,
            StandoffNuclear,
        };

        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let m_threat = threat_mass_kg();
        let v_esc = escape_speed_ms(m_threat, THREAT_RADIUS_M).expect("threat escape speed");
        let nuke = StandoffNuclear::DEARBORN_2007;
        // Δv per kilotonne on this body — the whole nuclear column in one number.
        let dv_per_kt = nuke.dv_ms(1.0, m_threat).expect("rate");
        // Interception geometry, stated not derived (the kinetic column depends on
        // it): DART's measured β, and a 10 km/s closing speed.
        let beta = 3.6_f64;
        let v_rel = 1.0e4_f64;

        println!(
            "threat: M = {m_threat:.3e} kg, r = {THREAT_RADIUS_M} m, v_esc = {v_esc:.4} m/s\n\
             nuclear rate = {:.4e} m/s per kt → {:.1} kt reaches escape speed\n\
             {:>10} {:>10} {:>11} {:>10} {:>9}  regime",
            dv_per_kt,
            v_esc / dv_per_kt,
            "lead (yr)",
            "Δv (m/s)",
            "yield (kt)",
            "mass (t)",
            "Δv/v_esc"
        );

        // Leads spanning the campaign, in orbital periods of the threat.
        let period = 24_928_208.624_301_072_f64;
        let mut all_disrupt = true;
        // The longest lead is also the cheapest Δv, so it doubles as the
        // "closest anyone gets to intact deflection" figure below. Captured from
        // the sweep rather than re-solved: each of these is a full-field
        // propagation costing ~90 s, and solving the same lead twice would be a
        // sixth of this test's runtime spent reproducing a number it already has.
        let mut easiest = f64::NAN;
        for periods in [0.5_f64, 1.0, 2.0, 4.0, 8.0] {
            let lead = periods * period;
            let dv = mc
                .required_dv_along_track(lead, SAFE_PERIGEE_TARGET_M)
                .expect("dv solve");
            easiest = dv;
            let yield_kt = nuke
                .yield_kilotonnes_for_dv(dv, m_threat)
                .expect("nuclear invert");
            let mass_kg =
                kinetic_impactor_mass_for_dv(dv, beta, v_rel, m_threat).expect("kinetic invert");
            let regime = disruption_regime(dv, m_threat, THREAT_RADIUS_M).expect("regime");
            println!(
                "{:>10.2} {dv:>10.4} {yield_kt:>11.1} {:>10.0} {:>9.3}  {regime:?}",
                lead / (365.25 * 86400.0),
                mass_kg / 1000.0,
                dv / v_esc
            );
            if regime != DisruptionRegime::LikelyDisruption {
                all_disrupt = false;
            }
        }

        // The claim, asserted rather than narrated. If a future force-model or
        // seed change ever moves this body out of the disruption regime, that is a
        // real change to the campaign's lesson and it should fail here first.
        assert!(
            all_disrupt,
            "every lead the campaign covers should require a Δv above this 300 m \
             body's own escape speed — a standoff burst that size disperses it \
             rather than deflecting it, which is why §5 puts the kinetic impactor \
             in the middle of the spectrum and this term at the top"
        );

        // And the crossover is quoted so the reader can see *how far* from intact
        // deflection this is: reaching `IntactDeflection` needs Δv ≤ 0.013·v_esc,
        // which is over two orders of magnitude below the easiest point on the
        // curve. Nothing in the campaign's lead-time range approaches it.
        let intact_ceiling = asteroid_core::deflection::INTACT_DV_OVER_VESC * v_esc;
        println!(
            "intact-deflection ceiling = {intact_ceiling:.5} m/s; easiest lead on the \
             curve still needs {easiest:.4} m/s = {:.0}x that",
            easiest / intact_ceiling
        );
        assert!(
            easiest > 10.0 * intact_ceiling,
            "the gap to intact nuclear deflection should be large and stated, not marginal"
        );
    }

    /// Kernel-gated (release-run). The single most decisive frame check: the
    /// threat *hits Earth* at the impact epoch by construction, so its heliocentric
    /// ecliptic-AU position there must coincide with Earth's to within the
    /// conditioned b-offset (~3000 km ≈ 2e-5 AU) plus round-trip integration error.
    /// This one assertion exercises the whole threat-frame chain end-to-end — the
    /// SSB→heliocentric subtraction, the m→km scaling, and the obliquity rotation:
    /// a missing Sun subtraction shows as a ~1e6 km gap, a m-vs-km slip as ~1000×.
    /// Far sharper than "distance ≈ a". Also pins the track sampler's length.
    #[test]
    fn asteroid_position_coincides_with_earth_at_impact() {
        if !have_kernels() {
            eprintln!("skipping asteroid_position_coincides_with_earth_at_impact: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let t_impact = mc.impact_tdb_seconds();
        let ast = mc
            .asteroid_position_ecl_au(t_impact)
            .expect("asteroid position at impact");
        let earth = mc
            .body_position_ecl_au(399, t_impact)
            .expect("earth position at impact");

        // Sane heliocentric band first — a wholly wrong frame (barycentric, or
        // km/m confusion) lands far outside this.
        assert!(
            (0.3..=3.0).contains(&ast.norm()),
            "threat heliocentric distance {:.4} AU is not in a sane band",
            ast.norm()
        );
        // The decisive coincidence: at impact the asteroid is on top of Earth.
        let gap_au = (ast - earth).norm();
        assert!(
            gap_au < 1.0e-3,
            "threat-Earth gap at impact {gap_au:.3e} AU too large — frame chain wrong \
             (Sun subtraction / km-vs-m / obliquity)"
        );

        // The track sampler returns exactly n points (no silent drops in-span).
        let track = mc.asteroid_track_ecl_au(200);
        assert_eq!(
            track.len(),
            200,
            "nominal track should be a full n-point line"
        );
        assert!(
            track.iter().all(|p| (0.2..=4.0).contains(&p.norm())),
            "every track point should sit at a plausible heliocentric distance"
        );
    }

    /// Kernel-gated (release-run). The capture radius is the bar every deflection
    /// verdict is measured against, so it has to mean what the planner claims it
    /// means: the nominal is a **hit** (perigee inside the focused disc), and the
    /// disc is the *focused* one, not solid Earth.
    ///
    /// The expected value is derived, not observed — which is the point, since a
    /// band fitted to whatever the code printed would ratify a bug. `v_rel_kms = 18`
    /// is the relative speed at the **impact point**, 3000 km from the geocentre and
    /// deep in Earth's well — *not* the speed at infinity. So:
    ///
    /// ```text
    ///   ε      = v²/2 − μ⊕/r   = 162 − 398600/3000 = 29.13 km²/s²
    ///   v_inf  = √(2ε)                             =  7.63 km/s
    ///   b_cap  = R⊕·√(1 + (v_esc/v_inf)²)          =  1.773 R⊕  ≈ 11 300 km
    /// ```
    ///
    /// (This is also exactly why the scenario module requires `v_rel ≥ ~15 km/s`:
    /// escape speed at 3000 km is 16.3 km/s, so a slower seed would not be
    /// hyperbolic there and the b-plane reduction would have nothing to reduce.)
    ///
    /// The band is tight around that derivation: 1.0 would mean focusing was
    /// dropped, and a materially different figure would mean `v_inf` — and with it
    /// every miss distance the planner reports — is not what we think it is.
    /// Without this, `capture_radius_m` is a number the frontend merely trusts.
    #[test]
    fn capture_radius_is_a_focused_disc_the_nominal_hit_falls_inside() {
        if !have_kernels() {
            eprintln!("skipping capture_radius_is_a_focused_disc_*: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        assert_eq!(
            mc.capture_radius_m(),
            None,
            "no capture radius before build"
        );
        assert_eq!(mc.nominal_perigee_m(), None);

        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let capture = mc.capture_radius_m().expect("capture radius after build");
        let perigee = mc.nominal_perigee_m().expect("nominal perigee after build");
        let r_earth = asteroid_core::geometry::EARTH_EQUATORIAL_RADIUS_M;

        // Focusing widens the collision cross-section well beyond the solid body:
        // v_inf ≈ 7.6 km/s against an 11.2 km/s escape speed, so the disc is ~1.77 R⊕
        // (see the derivation above). A real N-body encounter will not land exactly
        // on the two-body figure, hence a band rather than an equality.
        assert!(
            capture > r_earth,
            "capture radius {capture:.4e} m is not larger than R⊕ {r_earth:.4e} m — \
             gravitational focusing is missing"
        );
        assert!(
            (1.70..1.85).contains(&(capture / r_earth)),
            "capture radius is {:.3} R⊕, expected ~1.773 from v_inf ≈ 7.63 km/s — \
             either focusing is wrong or the encounter speed is not what the config says",
            capture / r_earth
        );

        // The whole scenario is a designed hit: the nominal must fall inside the
        // disc, or there is no impact for the player to deflect.
        assert!(
            perigee < capture,
            "nominal perigee {perigee:.4e} m is outside the capture radius \
             {capture:.4e} m — the nominal is not a hit"
        );
    }

    /// Kernel-gated (release-run). `threat_span_tdb` reports the window the threat
    /// can actually be looked up over, and that window is *narrow* — this is the
    /// gate the display hides the threat outside of.
    ///
    /// The test deliberately asserts the failure too: one second past the end, the
    /// position lookup returns `None`, which the binding marshals as `ZERO` — and
    /// `ZERO` in this heliocentric frame is the **Sun**. So an unhidden threat does
    /// not vanish outside its span, it renders sitting on the Sun. The clock clamp
    /// cannot save it: the clock is clamped to the kernel's ~300 years, while the
    /// span asserted here is ~12, so ~96% of the scrub range is outside it.
    #[test]
    fn threat_span_is_the_narrow_window_outside_which_a_lookup_is_the_sun() {
        if !have_kernels() {
            eprintln!("skipping threat_span_is_the_narrow_window_*: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        assert_eq!(mc.threat_span_tdb(), None, "no threat span before build");

        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");
        let (lo, hi) = mc.threat_span_tdb().expect("threat span after build");
        let cfg = ImpactorConfig::default();
        let epoch0 = cfg.epoch0().tdb_seconds_past_j2000();
        let impact = cfg.impact_epoch.tdb_seconds_past_j2000();

        // The span starts at the campaign epoch and runs past impact (the config's
        // 60-day margin), so the whole drawn campaign is inside it.
        assert!(
            (lo - epoch0).abs() < 1.0,
            "threat span starts at {lo}, expected the campaign epoch {epoch0}"
        );
        assert!(
            hi > impact,
            "threat span ends at {hi}, before impact at {impact} — the final \
             approach would be un-lookupable"
        );

        // Inside: a real position. Outside: nothing — which the frontend would draw
        // on the Sun. Both halves matter; the first alone would pass on a span that
        // silently covered everything.
        assert!(
            mc.asteroid_position_ecl_au(impact).is_some(),
            "the threat must resolve at impact, the one epoch that defines it"
        );
        assert_eq!(
            mc.asteroid_position_ecl_au(hi + 1.0),
            None,
            "a lookup one second past the span end must fail rather than return a \
             position — this is the ZERO-is-the-Sun trap the span gate exists for"
        );
        assert_eq!(
            mc.asteroid_position_ecl_au(lo - 1.0),
            None,
            "a lookup one second before the span start must likewise fail"
        );
    }

    /// Kernel-gated (release-run). The deflected surface obeys causality and the
    /// success-sentinel contract: before the deflection epoch the deflected
    /// position equals the nominal (the impulse has not acted yet); at impact it
    /// has moved; and exactly one of `is_clean_miss` / `deflected_perigee_m`
    /// carries the result (never both, never neither once a plan is set).
    #[test]
    fn deflected_surface_respects_causality_and_sentinels() {
        if !have_kernels() {
            eprintln!("skipping deflected_surface_respects_causality_and_sentinels: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        assert!(!mc.has_plan(), "no plan before set_plan");
        assert_eq!(mc.deflected_perigee_m(), None);
        assert!(!mc.is_clean_miss());

        // A modest along-track nudge one heliocentric period before impact.
        let lead = mc.period_seconds();
        mc.set_plan(lead, 0.1).expect("set_plan succeeds");
        assert!(mc.has_plan());

        let t_defl = mc.plan_deflection_tdb_seconds().expect("plan epoch");

        // Before the deflection epoch: deflected == nominal (no retroactive nudge).
        let t_before = t_defl - 1.0e6;
        let nom_before = mc
            .asteroid_position_ecl_au(t_before)
            .expect("nominal before defl");
        let defl_before = mc
            .deflected_position_ecl_au(t_before)
            .expect("deflected before defl");
        assert!(
            (nom_before - defl_before).norm() < 1.0e-12,
            "deflected position before the deflection epoch must equal the nominal"
        );

        // At impact: the deflected track has moved off the nominal.
        let t_impact = mc.impact_tdb_seconds();
        let nom_impact = mc
            .asteroid_position_ecl_au(t_impact)
            .expect("nominal at impact");
        let defl_impact = mc
            .deflected_position_ecl_au(t_impact)
            .expect("deflected at impact");
        assert!(
            (nom_impact - defl_impact).norm() > 1.0e-9,
            "a 0.1 m/s nudge one period out should visibly move the impact-epoch position"
        );

        // Sentinel contract: with a plan set, exactly one of the two reads the
        // outcome — a finite perigee XOR a clean (off-gate) miss.
        assert_ne!(
            mc.is_clean_miss(),
            mc.deflected_perigee_m().is_some(),
            "clean-miss and finite-perigee must be mutually exclusive with a plan set"
        );

        // The deflected track is a full n-point line too.
        assert_eq!(mc.deflected_track_ecl_au(150).len(), 150);
    }

    /// Kernel-gated (release-run). **The build worker's exact composition**, and the
    /// only thing that proves the comet reaches the display at all: `Mission`'s
    /// worker calls `BuiltScenario::build` → [`seed_orrery_body`] → [`install`], and
    /// no other test walks that sequence (`build_scenario` installs an empty
    /// catalog). Assembling it here rather than in GDScript is the point — a
    /// GDScript-only check would only say the flag flipped.
    ///
    /// The perihelion assertion is not decoration: `TRUE_ANOMALY_DEG` is *derived*
    /// from "round the Sun near the impact epoch" through a Kepler solve written
    /// out by hand in a doc comment. This re-measures that derivation on the real
    /// perturbed field, so a careless edit to the seed angle fails loudly instead of
    /// quietly parking the comet at aphelion for the whole campaign.
    /// **Real asteroids are scenery, and scenery cannot move the threat.**
    ///
    /// The sb441 mount had to prove this empirically, because mounting a kernel
    /// changes the almanac the threat is *integrated against* — and the check was
    /// real work (cap and |B| had to match pre-mount to the digit). For sampled
    /// NEOs the same claim holds for a stronger reason: a `.neo` table never
    /// reaches the almanac at all. It is read after the scenario is built, carries
    /// no gravitational parameter, and enters nothing but the catalog.
    ///
    /// So this test pins the structural version: **one** build, its threat numbers
    /// read before and after the asteroids are installed, compared with `==`
    /// rather than a tolerance. A tolerance here would be an admission that the
    /// scenery might perturb something a little.
    #[test]
    fn real_asteroids_join_the_catalog_without_touching_the_threat() {
        if !have_kernels() {
            eprintln!("skipping real_asteroids_join_the_catalog_*: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        let eph = mc.ephemeris_arc();
        let built = BuiltScenario::build(Arc::clone(&eph), &ImpactorConfig::default(), false)
            .expect("scenario builds");
        mc.install(built, Vec::new());

        // The threat, before any scenery exists.
        let (capture, perigee, impact) = (
            mc.capture_radius_m().expect("capture radius"),
            mc.nominal_perigee_m().expect("nominal perigee"),
            mc.impact_tdb_seconds(),
        );

        let neos = load_neo_bodies();
        if neos.is_empty() {
            // Loud, for the same reason the kernel skip is: a green run here that
            // installed nothing would be asserting nothing.
            assert!(
                !asteroid_core::kernels::require_kernels(),
                "ASTEROID_REQUIRE_KERNELS is set but no .neo tables loaded, so this \
                 test would have verified an empty catalog and printed green.\n{}",
                asteroid_core::horizons::not_found_message()
            );
            eprintln!("no .neo tables — skipping the real-asteroid half of this test");
            return;
        }
        let n_neos = neos.len();
        mc.adopt_bodies(neos);

        // Bit-identical, not "close". Nothing in the read path above touched the
        // field the threat was flown in.
        assert_eq!(mc.capture_radius_m(), Some(capture), "capture radius moved");
        assert_eq!(
            mc.nominal_perigee_m(),
            Some(perigee),
            "nominal perigee moved"
        );
        assert_eq!(mc.impact_tdb_seconds(), impact, "impact epoch moved");

        // And the asteroids are actually there, sampled, span-gated, and in NEO
        // territory — an empty or Sun-parked catalog would pass the checks above.
        assert_eq!(mc.catalog_count(), n_neos);
        for i in 0..mc.catalog_count() {
            let name = mc.catalog_name(i).expect("name").to_string();
            assert_eq!(mc.catalog_kind(i), Some("asteroid"), "{name}");
            assert_eq!(
                mc.catalog_provenance(i),
                Some("sampled"),
                "{name} must be labelled as JPL's trajectory, not ours"
            );

            let (lo, hi) = mc.catalog_span_tdb(i).expect("span");
            let r = mc
                .catalog_position_ecl_au(i, 0.5 * (lo + hi))
                .expect("in-span position")
                .norm();
            assert!(
                (0.1..5.0).contains(&r),
                "{name} sits {r:.3} AU from the Sun — a near-Earth asteroid does not"
            );

            // The span gate, per body. One day outside the table there is no
            // position, and the frontend must hide the body rather than draw the
            // ZERO that a lesser API would return here.
            assert!(
                mc.catalog_position_ecl_au(i, lo - 86_400.0).is_none(),
                "{name} answered before its table starts"
            );
            assert!(
                mc.catalog_position_ecl_au(i, hi + 86_400.0).is_none(),
                "{name} answered after its table ends"
            );

            // The polyline is what the orrery draws; an empty one is an invisible
            // asteroid and a short one is a broken arc.
            let track = mc.catalog_track_ecl_au(i, 256);
            assert_eq!(track.len(), 256, "{name} track");
            for p in &track {
                assert!(
                    (0.1..5.0).contains(&p.norm()),
                    "{name} track leaves NEO distances at {:.3} AU",
                    p.norm()
                );
            }
        }
    }

    /// Kernel-gated (release-run, **~70 s**: three ~16 s single-term propagations
    /// plus the build). The Tier-2 preview seam end to end — the on-demand numbers
    /// path that lights the frontend's GR/Yarkovsky/belt/SRP menu, walked the exact
    /// way the gdext worker walks it: build → install → `scenario_arc` →
    /// [`measure_tier2_shifts`] off the Arc clone → [`adopt_tier2_shifts`].
    ///
    /// Pins the three contracts that matter:
    /// - **Off until asked.** A freshly-installed scenario carries no shifts, so
    ///   [`has_tier2_preview`](MissionCore::has_tier2_preview) is `false` and every
    ///   term reads unavailable — the invariant that keeps the ~64 s off the build
    ///   critical path (the threat solution must not wait on the menu).
    /// - **Measured, the three always-available terms are real, finite, and actually
    ///   moved** the perigee off the baseline (a term that returned the baseline
    ///   unchanged would be a dead toggle).
    /// - **Belt is `None`, not `0`, when sb441 is unmounted.** This runs on the bare
    ///   DE almanac, so the belt shift is genuinely unavailable; surfacing it as a
    ///   `-1` sentinel rather than a `0` km "belt does nothing" is the whole reason
    ///   its field is an `Option`.
    #[test]
    fn tier2_preview_measures_every_kernel_free_term_and_leaves_belt_unavailable_unmounted() {
        if !have_kernels() {
            eprintln!("skipping tier2_preview_*: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        let eph = mc.ephemeris_arc();

        // Build and install exactly as the fast worker does — no preview on this path.
        let built = BuiltScenario::build(Arc::clone(&eph), &ImpactorConfig::default(), false)
            .expect("scenario builds");
        mc.install(built, Vec::new());
        assert!(
            !mc.has_tier2_preview(),
            "a freshly-built scenario carries no preview"
        );
        assert_eq!(
            mc.tier2_shifted_perigee_m("relativity"),
            None,
            "no preview yet → every term unavailable"
        );

        let baseline = mc.nominal_perigee_m().expect("baseline perigee");

        // Now the on-demand path: measure off an Arc clone of the installed scenario
        // (what the preview worker holds) and adopt the result — the ~80 s, paid only
        // when the operator opens the menu, never on the build.
        let scenario = mc
            .scenario_arc()
            .expect("installed scenario is Arc-shareable");
        let shifts = measure_tier2_shifts(&scenario, mc.small_bodies_mounted())
            .expect("tier2 preview measures");
        mc.adopt_tier2_shifts(shifts);
        assert!(
            mc.has_tier2_preview(),
            "adopted preview must light the menu"
        );

        // Every term except the belt: finite, and each genuinely moved the perigee off
        // the baseline (not a dead toggle). Driven off `TIER2_TERM_IDS` rather than a
        // list written here, so a term added to the preview cannot be missed — this
        // loop read `["relativity", "yarkovsky", "srp"]` while `J2` shipped, and was
        // green throughout.
        for term in TIER2_TERM_IDS.iter().filter(|t| **t != "belt") {
            let shifted = mc
                .tier2_shifted_perigee_m(term)
                .unwrap_or_else(|| panic!("{term} shift should be available"));
            assert!(
                shifted.is_finite() && shifted > 0.0,
                "{term} perigee {shifted} m"
            );
            assert!(
                (shifted - baseline).abs() > 1.0,
                "{term} left the perigee within 1 m of baseline ({shifted} vs {baseline}) — dead toggle"
            );
        }

        // Belt: unavailable (None → -1 at the FFI edge), NOT zero. The almanac has
        // no sb441, so the honest answer is "cannot say," never "does nothing."
        assert_eq!(
            mc.tier2_shifted_perigee_m("belt"),
            None,
            "belt shift must be unavailable (not 0) without the small-body kernel"
        );
        assert_eq!(
            mc.tier2_shifted_perigee_m("no-such-term"),
            None,
            "an unknown term is unavailable, not a silent 0"
        );
    }

    #[test]
    fn build_worker_installs_the_display_comet() {
        if !have_kernels() {
            eprintln!("skipping build_worker_installs_the_display_comet: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");

        // Exactly what `begin_build_scenario`'s worker does, in order.
        let eph = mc.ephemeris_arc();
        let built = BuiltScenario::build(Arc::clone(&eph), &ImpactorConfig::default(), false)
            .expect("scenario builds");
        let epoch0 = built.epoch0();
        let comet = seed_orrery_body(
            &eph,
            built.scenario_ref(),
            display_comet::NAME,
            display_comet::KIND,
            display_comet::elements(),
            epoch0,
            display_comet::CADENCE_SECONDS,
            display_comet::N_SNAPSHOTS,
        )
        .expect("comet flies in the built field");
        mc.install(built, vec![comet]);

        assert_eq!(mc.catalog_count(), 1);
        assert_eq!(mc.catalog_name(0), Some(display_comet::NAME));
        assert_eq!(mc.catalog_kind(0), Some(display_comet::KIND));

        // The span is the gate the display hides the comet outside of — one orbit.
        let epoch0_tdb = epoch0.tdb_seconds_past_j2000();
        let (lo, hi) = mc.catalog_span_tdb(0).expect("comet span");
        assert!((lo - epoch0_tdb).abs() < 1.0);
        let span_years = (hi - lo) / (365.25 * 86_400.0);
        assert!(
            (21.0..=24.0).contains(&span_years),
            "comet span {span_years:.1} yr is not the ~22.6 yr orbit it is authored as"
        );

        // Sweep the span: the comet stays on its designed ellipse, and its closest
        // approach to the Sun — the visible event — lands near the impact epoch.
        let impact_tdb = mc.impact_tdb_seconds();
        let (mut peri_r, mut peri_tdb) = (f64::INFINITY, 0.0);
        let samples = 4000;
        for k in 0..=samples {
            let tdb = lo + (hi - lo) * (k as f64 / samples as f64);
            let r = mc
                .catalog_position_ecl_au(0, tdb)
                .expect("in-span position")
                .norm();
            assert!(
                (0.7..=15.6).contains(&r),
                "comet at {r:.3} AU is off its designed ellipse [q, Q] ≈ [0.8, 15.2]"
            );
            if r < peri_r {
                peri_r = r;
                peri_tdb = tdb;
            }
        }
        assert!(
            (0.7..=0.95).contains(&peri_r),
            "perihelion {peri_r:.3} AU is not the designed q ≈ 0.8 AU"
        );
        let peri_vs_impact_yr = (peri_tdb - impact_tdb) / (365.25 * 86_400.0);
        assert!(
            peri_vs_impact_yr.abs() < 1.5,
            "perihelion falls {peri_vs_impact_yr:+.2} yr from impact — the seed angle no \
             longer puts the comet's pass anywhere near the campaign's payoff"
        );

        // The ZERO-is-the-Sun gate has something to gate on: outside the span the
        // read fails rather than silently returning the origin.
        assert!(mc.catalog_position_ecl_au(0, hi + 86_400.0).is_none());
        assert!(mc.catalog_position_ecl_au(0, lo - 86_400.0).is_none());
    }

    /// Kernel-gated (release-run). The orrery seed path is correct end-to-end. A
    /// synthetic body authored with **ecliptic** elements and **zero inclination**
    /// must (a) read back at its seed epoch as the *authored* heliocentric position
    /// — proving the ecliptic→ICRF→+Sun seed is the exact inverse of the read path
    /// — and (b) stay in the ecliptic plane (|z| ≈ 0) all along its integrated
    /// track, which it would NOT if the ecliptic↔ICRF rotation were wrong (a ~23°
    /// tilt would lift z by up to ~0.4·r). Also checks the orbit is physically on
    /// its designed ellipse (distance in `[a(1−e), a(1+e)]`) and the metadata.
    #[test]
    fn synthetic_body_seeds_and_frames_correctly() {
        if !have_kernels() {
            eprintln!("skipping synthetic_body_seeds_and_frames_correctly: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        // Adding a body before a scenario is built is an error, not a panic.
        // (Re-checked here since build already ran; use a fresh core for the guard.)
        let mut unbuilt = MissionCore::load().expect("load kernels");
        let planar = OrbitalElements::new(2.0 * AU_M, 0.2, 0.0, 0.0, 0.0, 0.0);
        assert!(unbuilt
            .add_synthetic_body("X", "asteroid", planar, epoch_2035(), 5.0 * 86_400.0, 4)
            .is_err());

        let a_m = 2.0 * AU_M;
        let e = 0.2;
        let elements = OrbitalElements::new(a_m, e, 0.0, 0.0, 0.0, 0.0); // ecliptic, planar
        let epoch0 = epoch_2035();
        let epoch0_tdb = epoch0.tdb_seconds_past_j2000();
        let cadence = 5.0 * 86_400.0;
        let n = 146; // ~2 years — most of one orbit (T = 2^1.5 ≈ 2.83 yr)

        // The authored heliocentric ecliptic position, in AU, for the round-trip.
        let mu_sun = mc.ephemeris.sun_gm_m3_s2().expect("sun GM");
        let expected_ecl_au = elements.to_state(mu_sun).position / AU_M;

        let idx = mc
            .add_synthetic_body("TEST-COMET", "comet", elements, epoch0, cadence, n)
            .expect("add synthetic body");
        assert_eq!(idx, 0);
        assert_eq!(mc.catalog_count(), 1);
        assert_eq!(mc.catalog_name(idx), Some("TEST-COMET"));
        assert_eq!(mc.catalog_kind(idx), Some("comet"));

        // (a) Seed round-trip: at epoch0 the read recovers the authored position.
        let at0 = mc
            .catalog_position_ecl_au(idx, epoch0_tdb)
            .expect("position at seed epoch");
        assert!(
            (at0 - expected_ecl_au).norm() < 1e-6,
            "seed round-trip off by {:.3e} AU — ecliptic↔ICRF seed/read not inverse",
            (at0 - expected_ecl_au).norm()
        );

        // Span covers [epoch0, epoch0 + n·cadence]; used to clamp/hide the body.
        let (lo, hi) = mc.catalog_span_tdb(idx).expect("span");
        assert!((lo - epoch0_tdb).abs() < 1.0);
        assert!((hi - (epoch0_tdb + cadence * n as f64)).abs() < 1.0);

        // (b) Planarity + on-ellipse across the whole track.
        let track = mc.catalog_track_ecl_au(idx, 200);
        assert_eq!(track.len(), 200, "track should be a full n-point line");
        for p in &track {
            assert!(
                p.z.abs() < 0.02,
                "planar (i=0) ecliptic orbit lifted to |z| = {:.4} AU — rotation wrong",
                p.z.abs()
            );
            assert!(
                (1.55..=2.45).contains(&p.norm()),
                "distance {:.4} AU outside the designed ellipse [a(1−e), a(1+e)]",
                p.norm()
            );
        }

        // Out-of-range index and out-of-span epoch both return None (no panic).
        assert!(mc.catalog_position_ecl_au(9, epoch0_tdb).is_none());
        assert!(mc
            .catalog_position_ecl_au(idx, epoch0_tdb - 1.0e9)
            .is_none());
    }

    /// Kernel-gated (release-run). **The decisive test for the b-plane view**: the
    /// projected tracks and the projected b-point have to be in the *same frame*,
    /// and the assertion below is what proves it on real data rather than by
    /// inspection.
    ///
    /// Far from Earth the asteroid is on its incoming asymptote, and the asymptote's
    /// defining property is that it pierces the b-plane exactly at `B`. So the very
    /// first sample of the track — ~1.5 days out, beyond Earth's sphere of influence
    /// — must have (ξ, ζ) ≈ the b-point's (ξ, ζ). It is the *transverse* components
    /// that must agree; `s` is enormous and negative there, which is precisely what
    /// gives this test its teeth: the far sample sits ~10⁶ km down the `s` axis
    /// against a `|B|` of ~10⁴ km, so a frame error of the obliquity's 23.4° would
    /// spill `sin(23.4°) × 10⁶ ≈ 4×10⁵` km of depth into the plotted plane — a ~50×
    /// blowout of a tolerance set at a fraction of `|B|`. That is the exact mistake
    /// this guards: running the tracks through `icrf_km_to_ecliptic_au` (right for
    /// the orrery, wrong here) while `Ŝ` and `B` stay ICRF. Nothing would error; the
    /// plot would just be quietly, plausibly wrong.
    ///
    /// Also pinned: `|B|` survives the projection (it is a rotation), `B` lands *in*
    /// the b-plane (`s ≈ 0`, since `B ⊥ Ŝ` by construction), `s` sweeps
    /// monotonically from inbound to outbound, and the empty-vs-zeroed contract at
    /// both gates (no scenario → no track; no plan → no deflected track).
    #[test]
    fn the_encounter_projects_into_one_frame_the_asymptote_pierces_where_b_says() {
        if !have_kernels() {
            eprintln!("skipping the_encounter_projects_into_one_frame_*: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");

        // Before the build there is no frame and nothing to draw — not a zeroed one.
        assert!(mc.encounter_nominal_track_km().is_empty());
        assert!(mc.encounter_deflected_track_km().is_empty());
        assert!(mc.nominal_b_point_km().is_none());
        assert!(mc.encounter_sample_span_tdb().is_none());

        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let track = mc.encounter_nominal_track_km();
        assert_eq!(
            track.len(),
            ENCOUNTER_SAMPLES,
            "the nominal track must be available with no plan and no propagation"
        );
        assert!(
            mc.encounter_deflected_track_km().is_empty(),
            "no plan means NO deflected track — an empty one, not a zeroed one that \
             would draw the asteroid through Earth's centre"
        );

        let b_point = mc.nominal_b_point_km().expect("b-point after build");
        let b = mc.nominal_impact_parameter_m().expect("|B| after build") / M_PER_KM;

        // The projection is a rotation: |B| is preserved.
        assert!(
            (b_point.norm() - b).abs() / b < 1e-9,
            "projected |B| {:.3} km ≠ impact parameter {b:.3} km",
            b_point.norm()
        );
        // B lies in the b-plane: its depth along the asymptote is zero.
        assert!(
            b_point.z.abs() / b < 1e-9,
            "the b-point has depth s = {:.3} km along Ŝ; B ⊥ Ŝ by construction",
            b_point.z
        );

        // s sweeps inbound (negative) → outbound (positive), strictly.
        let (s_first, s_last) = (track[0].z, track[track.len() - 1].z);
        assert!(
            s_first < 0.0 && s_last > 0.0,
            "the window must straddle the b-plane: s runs {s_first:.3e} → {s_last:.3e} km"
        );
        assert!(
            track.windows(2).all(|w| w[1].z > w[0].z),
            "depth along the incoming asymptote must increase monotonically"
        );

        // THE assertion. Far out, the track is on the asymptote, which pierces the
        // b-plane at B — so the transverse components must already agree there.
        let far = track[0];
        let transverse_gap = ((far.x - b_point.x).powi(2) + (far.y - b_point.y).powi(2)).sqrt();
        assert!(
            transverse_gap < 0.25 * b,
            "the far-field track sample sits {transverse_gap:.1} km from the b-point in \
             the plotted plane (|B| = {b:.1} km, depth s = {:.3e} km). The asymptote \
             must pierce the b-plane AT B — a gap this size means the tracks and Ŝ/B \
             are not in the same frame (an obliquity mix-up would show ~{:.1e} km here)",
            far.z,
            far.z.abs() * (23.4_f64.to_radians()).sin()
        );

        // BOTH marks must plot at exactly their own stated |B| — the property the
        // whole view rests on, since "outside the dashed disc" and "the panel says
        // MISS" are the same claim and a player sees them together. The nominal gets
        // this free (B ⊥ its own Ŝ); the deflected B belongs to a *different*
        // b-plane, so it is rescaled (see `deflected_b_point_km`) and this is what
        // pins that. Asserted on the plotted radius — the ξ/ζ the view actually
        // draws — not on the 3-vector's norm, which would pass either way.
        mc.set_plan(mc.period_seconds(), -0.2).expect("plan solves");
        for (point, b_m, who) in [
            (
                mc.nominal_b_point_km(),
                mc.nominal_impact_parameter_m(),
                "nominal",
            ),
            (
                mc.deflected_b_point_km(),
                mc.deflected_impact_parameter_m(),
                "deflected",
            ),
        ] {
            let p = point.unwrap_or_else(|| panic!("{who}: no b-point"));
            let b_km = b_m.unwrap_or_else(|| panic!("{who}: no |B|")) / M_PER_KM;
            let plotted = (p.x * p.x + p.y * p.y).sqrt();
            assert!(
                (plotted - b_km).abs() / b_km < 1e-9,
                "{who} b-point plots at {plotted:.3} km but its |B| is {b_km:.3} km — the \
                 mark and the number a player reads together must be the same distance, \
                 or the picture can put it inside the disc while the panel says MISS"
            );
        }

        // The sample span is the window the core defines, centred on impact.
        let (lo, hi) = mc.encounter_sample_span_tdb().expect("sample span");
        assert!(
            ((hi - lo) - 2.0 * ENCOUNTER_HALF_WINDOW_SECONDS).abs() < 1.0,
            "sample span {:.1} s ≠ the core's ±{:.1} s window",
            hi - lo,
            ENCOUNTER_HALF_WINDOW_SECONDS
        );
    }

    /// Kernel-gated (release-run). **The verdict is `b` against the capture radius**
    /// — the pair the core's own `is_hit` compares — and this pins the frontend's
    /// comparison to it on a real N-body encounter.
    ///
    /// There are exactly two coherent hit criteria, and they are equivalent:
    /// `b > b_capture` (the un-focused asymptotic miss against the target enlarged
    /// for focusing) and `perigee > R⊕` (the already-focused closest approach
    /// against the solid body). Both are asserted here against `is_hit`, which also
    /// makes this the first check that the core's two-body equivalence survives
    /// contact with the full perturbed field.
    ///
    /// The mistake this exists to prevent is mixing them — testing `perigee >
    /// b_capture`, which is neither pair. It reads plausible (both are "miss
    /// distances", both are in metres) and it is silently ~1.5× too strict, so it
    /// fails a plan that physics calls safe. The final assertion measures that
    /// factor from the encounter's own μ and v_inf rather than trusting the claim.
    #[test]
    fn the_hit_criterion_is_b_against_the_capture_disc_not_the_perigee() {
        if !have_kernels() {
            eprintln!("skipping the_hit_criterion_is_b_against_the_capture_disc_*: no DE kernel");
            return;
        }
        let mut mc = MissionCore::load().expect("load kernels");
        mc.build_scenario(&ImpactorConfig::default())
            .expect("scenario builds");

        let capture = mc.capture_radius_m().expect("capture radius");
        let r_earth = mc.earth_radius_m().expect("Earth radius");
        let v_inf = mc.encounter_v_inf_m_s().expect("v_inf");

        // The nominal is the designed hit, under both criteria.
        let b_nom = mc.nominal_impact_parameter_m().expect("nominal |B|");
        let p_nom = mc.nominal_perigee_m().expect("nominal perigee");
        assert!(
            b_nom < capture && p_nom < r_earth,
            "the nominal must be a hit both ways: b {b_nom:.4e} vs capture {capture:.4e}, \
             perigee {p_nom:.4e} vs R⊕ {r_earth:.4e}"
        );

        // A plan chosen to land in the band where the two bars actually DISAGREE —
        // a 0.2 m/s nudge one period before impact. Measured: b ≈ 14 640 km,
        // perigee ≈ 9 319 km, capture ≈ 11 311 km. So b > capture (a miss) while
        // perigee < capture (the mixed bar's "hit"). This is not a contrived corner:
        // it is a plan a player can dial in, and on it the old comparison printed
        // SURFACE IMPACT over a pass that physics says clears Earth by 2 941 km.
        mc.set_plan(mc.period_seconds(), -0.2).expect("plan solves");
        let enc = mc
            .plan
            .as_ref()
            .expect("plan")
            .encounter
            .expect("this nudge should leave a finite-perigee encounter");
        let b = mc.deflected_impact_parameter_m().expect("deflected |B|");
        let perigee = mc.deflected_perigee_m().expect("deflected perigee");

        assert_eq!(
            b > capture,
            !enc.is_hit(),
            "the frontend's comparison (b {b:.4e} > capture {capture:.4e}) disagrees with \
             the core's own is_hit()"
        );
        assert_eq!(
            perigee > r_earth,
            !enc.is_hit(),
            "the other coherent pair (perigee {perigee:.4e} > R⊕ {r_earth:.4e}) disagrees \
             with is_hit() — the two-body equivalence does not survive the real field"
        );

        // b is the asymptotic miss and the perigee is the focused one, so b > perigee
        // always; and the capture disc is larger than the solid body. Together those
        // are why `perigee > capture` is a *third*, stricter bar rather than a typo
        // that happens to work.
        assert!(
            b > perigee,
            "b {b:.4e} must exceed the perigee {perigee:.4e} it focuses down to"
        );
        assert!(capture > r_earth, "the capture disc must exceed R⊕");

        // How much stricter, measured rather than asserted from memory: the b that
        // corresponds to a perigee of exactly `capture` (via b² = r_p² + 2μr_p/v_inf²).
        // The honest bar is b > capture; the mixed bar is b > this.
        let b_at_perigee_capture =
            (capture * capture + 2.0 * enc.mu * capture / (v_inf * v_inf)).sqrt();
        assert!(
            b_at_perigee_capture > 1.3 * capture,
            "expected `perigee > capture` to be substantially stricter than `b > capture` \
             ({b_at_perigee_capture:.4e} vs {capture:.4e} m) — if these have converged, the \
             focusing is gone and the whole encounter is wrong"
        );

        // And the equivalence read the other way: the b at a perigee of exactly R⊕ IS
        // the capture radius. This is the identity that makes the two pairs one test.
        let b_at_perigee_r_earth =
            (r_earth * r_earth + 2.0 * enc.mu * r_earth / (v_inf * v_inf)).sqrt();
        assert!(
            (b_at_perigee_r_earth - capture).abs() / capture < 1e-9,
            "b at perigee = R⊕ is {b_at_perigee_r_earth:.6e} m but the capture radius is \
             {capture:.6e} m — these are the same number by definition"
        );

        // The bug, pinned on the very plan that exposes it. This nudge is a genuine
        // miss — both coherent pairs say so, and `is_hit` agrees — yet the mixed bar
        // `perigee > capture` calls it a hit. Asserting the *disagreement* rather
        // than only the fix is what makes this a regression test: bring the old
        // comparison back anywhere and this fails, naming a plan it lies about.
        assert!(
            !enc.is_hit(),
            "this plan is supposed to be a miss; the band it was chosen for has moved"
        );
        assert!(
            perigee <= capture,
            "expected this plan to sit in the disagreement band (perigee {perigee:.4e} < \
             capture {capture:.4e} < b {b:.4e}) — that band is the whole point of the \
             test, and without it nothing here would notice the mixed bar coming back"
        );
    }
}
