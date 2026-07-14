//! `asteroid_gdext` — the GDExtension binding that exposes the headless,
//! deterministic [`asteroid_core`] physics to the Godot Phase-2 frontend.
//!
//! **Dependency direction is one-way:** this crate depends on `asteroid_core`;
//! no Godot type ever links back into the core (HANDOFF §10 invariant — the
//! core stays renderer-free so it remains the single validated source of truth).
//!
//! [`AsteroidCore`] is **Commit 1: the toolchain gate** — one class returning the
//! core version string, proving GDExtension class registration, the Rust↔Godot
//! FFI boundary, and that a gdext build loads in Godot 4.7 (runtime ≥ API
//! forward-compat). [`Mission`] is **Commit 2**: the real scenario surface — real
//! DE440 body positions for the display and the along-track Δv the planner needs,
//! all delegating to the godot-free [`mission_core::MissionCore`] so the logic
//! stays unit-testable without a running Godot. Every `#[func]` is panic-free:
//! a missing kernel or a failed lookup becomes a status/return value, never a
//! panic across the FFI boundary.

mod mission_core;

use godot::prelude::*;

use asteroid_core::scenario::ImpactorConfig;
use asteroid_core::{Epoch, OrbitalElements};
use mission_core::MissionCore;

/// Metres per astronomical unit — synthetic-body semi-major axes reach the SI
/// core as AU from GDScript.
const AU_M: f64 = 1.495_978_707e11;

struct AsteroidGdext;

#[gdextension]
unsafe impl ExtensionLibrary for AsteroidGdext {}

/// Thin handle onto the Rust core, registered with Godot as `AsteroidCore`.
///
/// `RefCounted` so GDScript can `AsteroidCore.new()` and let it free itself —
/// no manual lifetime management on the script side.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
struct AsteroidCore {
    base: Base<RefCounted>,
}

#[godot_api]
impl AsteroidCore {
    /// The `asteroid_core` crate version (`CARGO_PKG_VERSION`) — the load-gate
    /// round trip. If GDScript reads this string back, the binding is live.
    #[func]
    fn core_version(&self) -> GString {
        asteroid_core::CORE_VERSION.into()
    }
}

/// The real mission, exposed to GDScript as `Mission`. A thin marshalling shell
/// over [`MissionCore`]: every method maps a core `Result`/`Option` to a plain
/// return value or a `false`/zero/`-1` sentinel, so nothing panics across FFI.
///
/// Two-phase, mirroring [`MissionCore`]: [`load`](Self::load) reads the kernels
/// (fast → body positions available) and [`build_scenario`](Self::build_scenario)
/// runs the expensive back-propagation (→ the Δv solver). Kernel-missing surfaces
/// through [`last_error`](Self::last_error) for the HUD.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
struct Mission {
    core: Option<MissionCore>,
    error: GString,
    base: Base<RefCounted>,
}

#[godot_api]
impl Mission {
    /// Load the DE440 kernels (env-var paths). Returns `true` on success; on
    /// failure returns `false` and stores the reason in [`last_error`](Self::last_error)
    /// (e.g. "environment variable ASTEROID_DE_KERNEL is not set"). Fast.
    #[func]
    fn load(&mut self) -> bool {
        match MissionCore::load() {
            Ok(c) => {
                self.core = Some(c);
                self.error = GString::new();
                true
            }
            Err(e) => {
                self.error = e.to_string().as_str().into();
                self.core = None;
                false
            }
        }
    }

    /// Build the designer impactor + campaign (the expensive multi-year
    /// back-propagation). Must be called after [`load`](Self::load). Returns
    /// `true` on success; `false` + [`last_error`](Self::last_error) otherwise.
    #[func]
    fn build_scenario(&mut self) -> bool {
        let Some(core) = self.core.as_mut() else {
            self.error = "load() must succeed before build_scenario()".into();
            return false;
        };
        match core.build_scenario(&ImpactorConfig::default()) {
            Ok(()) => {
                self.error = GString::new();
                true
            }
            Err(e) => {
                self.error = e.to_string().as_str().into();
                false
            }
        }
    }

    /// Whether the kernels are loaded (body positions available).
    #[func]
    fn is_loaded(&self) -> bool {
        self.core.is_some()
    }

    /// Whether the scenario is built (the Δv solver is available).
    #[func]
    fn is_ready(&self) -> bool {
        self.core.as_ref().is_some_and(|c| c.has_scenario())
    }

    /// The reason the last `load`/`build_scenario` failed (empty if none).
    #[func]
    fn last_error(&self) -> GString {
        self.error.clone()
    }

    /// `"debug"` or `"release"` — which build profile this loaded DLL is, so the
    /// frontend/tests can tell (the real scenario path is only usable in release).
    #[func]
    fn build_profile(&self) -> GString {
        if cfg!(debug_assertions) {
            "debug".into()
        } else {
            "release".into()
        }
    }

    /// Heliocentric **ecliptic-J2000** position of NAIF body `naif_id` at
    /// `tdb_seconds` past J2000, in **AU** (a Godot `Vector3`; f32 is ample at AU
    /// scale). `Vector3::ZERO` if not loaded or the lookup fails. The GDScript
    /// side maps ecliptic AU → scene units with its existing `ecl_to_godot`.
    #[func]
    fn body_position_ecl_au(&self, naif_id: i64, tdb_seconds: f64) -> Vector3 {
        match self
            .core
            .as_ref()
            .and_then(|c| c.body_position_ecl_au(naif_id as i32, tdb_seconds))
        {
            Some(v) => Vector3::new(v.x as f32, v.y as f32, v.z as f32),
            None => Vector3::ZERO,
        }
    }

    /// Minimum along-track Δv (m/s) to lift the b-plane perigee to
    /// `target_perigee_m`, applied `lead_seconds` before impact. `-1.0` if the
    /// scenario is not built or the solve fails.
    #[func]
    fn required_dv_along_track(&self, lead_seconds: f64, target_perigee_m: f64) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| {
                c.required_dv_along_track(lead_seconds, target_perigee_m)
                    .ok()
            })
            .unwrap_or(-1.0)
    }

    /// Heliocentric semi-major axis of the threat, m (0 if no scenario).
    #[func]
    fn semi_major_axis_m(&self) -> f64 {
        self.core.as_ref().map_or(0.0, |c| c.semi_major_axis_m())
    }

    /// Heliocentric orbital period of the threat, seconds (0 if no scenario).
    #[func]
    fn period_seconds(&self) -> f64 {
        self.core.as_ref().map_or(0.0, |c| c.period_seconds())
    }

    /// Impact epoch, seconds past J2000 (0 if no scenario).
    #[func]
    fn impact_tdb_seconds(&self) -> f64 {
        self.core.as_ref().map_or(0.0, |c| c.impact_tdb_seconds())
    }

    /// Campaign-start epoch, seconds past J2000 (0 if no scenario).
    #[func]
    fn epoch0_tdb_seconds(&self) -> f64 {
        self.core.as_ref().map_or(0.0, |c| c.epoch0_tdb_seconds())
    }

    /// Nominal (un-deflected) threat position at `tdb_seconds`, heliocentric
    /// **ecliptic AU** — the same display frame as
    /// [`body_position_ecl_au`](Self::body_position_ecl_au), so the drawn asteroid
    /// sits on the drawn planets' orbits. `Vector3::ZERO` before the scenario is
    /// built or outside the propagated span.
    #[func]
    fn asteroid_position_ecl_au(&self, tdb_seconds: f64) -> Vector3 {
        match self
            .core
            .as_ref()
            .and_then(|c| c.asteroid_position_ecl_au(tdb_seconds))
        {
            Some(v) => Vector3::new(v.x as f32, v.y as f32, v.z as f32),
            None => Vector3::ZERO,
        }
    }

    /// Deflected threat position at `tdb_seconds`, heliocentric **ecliptic AU**.
    /// Equals the nominal position before the plan's deflection epoch (no
    /// retroactive nudge). `Vector3::ZERO` if no plan is set or the epoch is out
    /// of span.
    #[func]
    fn deflected_position_ecl_au(&self, tdb_seconds: f64) -> Vector3 {
        match self
            .core
            .as_ref()
            .and_then(|c| c.deflected_position_ecl_au(tdb_seconds))
        {
            Some(v) => Vector3::new(v.x as f32, v.y as f32, v.z as f32),
            None => Vector3::ZERO,
        }
    }

    /// The nominal threat orbit as `samples` heliocentric ecliptic-AU points from
    /// campaign start to impact — the polyline the display draws. Sample **once**
    /// (it walks the whole span). Empty if no scenario.
    #[func]
    fn asteroid_track_ecl_au(&self, samples: i64) -> PackedVector3Array {
        let n = samples.max(0) as usize;
        let pts = self
            .core
            .as_ref()
            .map(|c| c.asteroid_track_ecl_au(n))
            .unwrap_or_default();
        let mut arr = PackedVector3Array::new();
        for v in pts {
            arr.push(Vector3::new(v.x as f32, v.y as f32, v.z as f32));
        }
        arr
    }

    /// The deflected threat orbit as `samples` heliocentric ecliptic-AU points
    /// (nominal up to the deflection epoch, deflected after). Empty if no plan is
    /// set. Re-sample after [`set_plan`](Self::set_plan).
    #[func]
    fn deflected_track_ecl_au(&self, samples: i64) -> PackedVector3Array {
        let n = samples.max(0) as usize;
        let pts = self
            .core
            .as_ref()
            .map(|c| c.deflected_track_ecl_au(n))
            .unwrap_or_default();
        let mut arr = PackedVector3Array::new();
        for v in pts {
            arr.push(Vector3::new(v.x as f32, v.y as f32, v.z as f32));
        }
        arr
    }

    /// Commit a deflection plan: an along-track impulse of `dv_along_track` (m/s)
    /// applied `lead_seconds` before impact. Returns `true` on success; on failure
    /// returns `false` and stores the reason in [`last_error`](Self::last_error).
    /// **Expensive** (re-propagates) — call on a plan change, not per frame.
    #[func]
    fn set_plan(&mut self, lead_seconds: f64, dv_along_track: f64) -> bool {
        let Some(core) = self.core.as_mut() else {
            self.error = "load()/build_scenario() must succeed before set_plan()".into();
            return false;
        };
        match core.set_plan(lead_seconds, dv_along_track) {
            Ok(()) => {
                self.error = GString::new();
                true
            }
            Err(e) => {
                self.error = e.to_string().as_str().into();
                false
            }
        }
    }

    /// Whether a deflection plan is currently set.
    #[func]
    fn has_plan(&self) -> bool {
        self.core.as_ref().is_some_and(|c| c.has_plan())
    }

    /// Whether the current plan produces a clean, wide miss (the deflected pass
    /// left the scan gate) — the **success** case, distinct from "no plan". When
    /// this is `true`, [`deflected_perigee_m`](Self::deflected_perigee_m) is `-1`
    /// because there is no finite perigee to report.
    #[func]
    fn is_clean_miss(&self) -> bool {
        self.core.as_ref().is_some_and(|c| c.is_clean_miss())
    }

    /// The deflected b-plane perigee (miss distance), m. `-1.0` if no plan is set
    /// **or** the pass is a clean miss — distinguish those with
    /// [`has_plan`](Self::has_plan) / [`is_clean_miss`](Self::is_clean_miss).
    #[func]
    fn deflected_perigee_m(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.deflected_perigee_m())
            .unwrap_or(-1.0)
    }

    /// The current plan's deflection epoch, seconds past J2000 (`-1` if no plan).
    #[func]
    fn plan_deflection_tdb_seconds(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.plan_deflection_tdb_seconds())
            .unwrap_or(-1.0)
    }

    // --- Orrery catalog: multiple bodies, long spans, cheap scrub --------------

    /// Add a synthetic designer body to the orrery and return its index (`-1` on
    /// failure, with the reason in [`last_error`](Self::last_error)). Orbit given
    /// by ecliptic Keplerian elements — `a_au` (AU), `e`, and the angles in
    /// **degrees** — valid at `epoch0_tdb_seconds`, then integrated once through
    /// the real field over `span_days` at `cadence_days` snapshots. Requires
    /// [`build_scenario`](Self::build_scenario). **Expensive** (one integration);
    /// call at load, not per frame.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn add_synthetic_body(
        &mut self,
        name: GString,
        kind: GString,
        a_au: f64,
        e: f64,
        incl_deg: f64,
        raan_deg: f64,
        argp_deg: f64,
        true_anomaly_deg: f64,
        epoch0_tdb_seconds: f64,
        span_days: f64,
        cadence_days: f64,
    ) -> i64 {
        let Some(core) = self.core.as_mut() else {
            self.error = "load()/build_scenario() must succeed before add_synthetic_body()".into();
            return -1;
        };
        // Validate the orbit up front so nothing panics across the FFI boundary
        // (an out-of-range inclination would trip the core's debug_assert, a
        // non-elliptical e would produce a nonsense state).
        if !(a_au.is_finite() && a_au > 0.0)
            || !(0.0..1.0).contains(&e)
            || !(0.0..=180.0).contains(&incl_deg)
            || !(cadence_days.is_finite() && cadence_days > 0.0)
            || !(span_days.is_finite() && span_days > 0.0)
        {
            self.error =
                "invalid orbit: need a_au>0, 0<=e<1, incl in [0,180] deg, span/cadence>0".into();
            return -1;
        }
        let elements = OrbitalElements::new(
            a_au * AU_M,
            e,
            incl_deg.to_radians(),
            raan_deg.to_radians(),
            argp_deg.to_radians(),
            true_anomaly_deg.to_radians(),
        );
        let epoch0 = Epoch::from_tdb_seconds_past_j2000(epoch0_tdb_seconds);
        let cadence_seconds = cadence_days * 86_400.0;
        let n_snapshots = (span_days / cadence_days).ceil().max(1.0) as u32;
        match core.add_synthetic_body(
            &name.to_string(),
            &kind.to_string(),
            elements,
            epoch0,
            cadence_seconds,
            n_snapshots,
        ) {
            Ok(idx) => {
                self.error = GString::new();
                idx as i64
            }
            Err(e) => {
                self.error = e.to_string().as_str().into();
                -1
            }
        }
    }

    /// Number of bodies in the orrery catalog.
    #[func]
    fn catalog_count(&self) -> i64 {
        self.core.as_ref().map_or(0, |c| c.catalog_count() as i64)
    }

    /// Display label of catalog body `index` (empty string if out of range).
    #[func]
    fn catalog_name(&self, index: i64) -> GString {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.core.as_ref().and_then(|c| c.catalog_name(i)))
            .map_or_else(GString::new, |s| s.into())
    }

    /// Coarse class of catalog body `index` (`"asteroid"`/`"comet"`/…; empty if
    /// out of range).
    #[func]
    fn catalog_kind(&self, index: i64) -> GString {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.core.as_ref().and_then(|c| c.catalog_kind(i)))
            .map_or_else(GString::new, |s| s.into())
    }

    /// Position of catalog body `index` at `tdb_seconds`, heliocentric **ecliptic
    /// AU** (the planets' frame). `Vector3::ZERO` if the index is invalid or the
    /// epoch is outside the body's propagated span (use
    /// [`catalog_span_tdb`](Self::catalog_span_tdb) to know which).
    #[func]
    fn catalog_position_ecl_au(&self, index: i64, tdb_seconds: f64) -> Vector3 {
        match usize::try_from(index).ok().and_then(|i| {
            self.core
                .as_ref()
                .and_then(|c| c.catalog_position_ecl_au(i, tdb_seconds))
        }) {
            Some(v) => Vector3::new(v.x as f32, v.y as f32, v.z as f32),
            None => Vector3::ZERO,
        }
    }

    /// Catalog body `index`'s orbit as `samples` heliocentric ecliptic-AU points
    /// across its whole propagated span — the polyline. Sample **once**. Empty if
    /// the index is invalid.
    #[func]
    fn catalog_track_ecl_au(&self, index: i64, samples: i64) -> PackedVector3Array {
        let n = samples.max(0) as usize;
        let pts = usize::try_from(index)
            .ok()
            .map(|i| {
                self.core
                    .as_ref()
                    .map(|c| c.catalog_track_ecl_au(i, n))
                    .unwrap_or_default()
            })
            .unwrap_or_default();
        let mut arr = PackedVector3Array::new();
        for v in pts {
            arr.push(Vector3::new(v.x as f32, v.y as f32, v.z as f32));
        }
        arr
    }

    /// Catalog body `index`'s propagated span as `[lo, hi]` seconds past J2000 (a
    /// 2-element array; **empty** if the index is invalid). f64 precision, unlike a
    /// `Vector2`, because a TDB second near 1e9 would lose ~64 s as f32. The
    /// frontend clamps/hides the body outside this window.
    #[func]
    fn catalog_span_tdb(&self, index: i64) -> PackedFloat64Array {
        let mut arr = PackedFloat64Array::new();
        if let Some((lo, hi)) = usize::try_from(index)
            .ok()
            .and_then(|i| self.core.as_ref().and_then(|c| c.catalog_span_tdb(i)))
        {
            arr.push(lo);
            arr.push(hi);
        }
        arr
    }
}
