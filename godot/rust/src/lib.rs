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

use std::sync::{mpsc, Arc};

use godot::prelude::*;

use asteroid_core::scenario::{ImpactorConfig, ScenarioError};
use asteroid_core::{Epoch, OrbitalElements};
use mission_core::{
    display_comet, load_neo_bodies, mount_small_bodies, seed_orrery_body, BuiltScenario,
    MissionCore, OrreryBody,
    SB441_BODIES,
};

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
    /// The in-flight background scenario build, if any — see
    /// [`begin_build_scenario`](Mission::begin_build_scenario). `Some` exactly while
    /// a worker is running, so it doubles as the "is building" flag.
    build: Option<mpsc::Receiver<Result<(BuiltScenario, Vec<OrreryBody>), String>>>,
    error: GString,
    base: Base<RefCounted>,
}

#[godot_api]
impl Mission {
    /// Load the DE440 kernels from the `ASTEROID_DE_KERNEL` /
    /// `ASTEROID_PLANETARY_CONSTANTS` env vars. Returns `true` on success; on
    /// failure returns `false` and stores the reason in
    /// [`last_error`](Self::last_error). Fast.
    ///
    /// **A launched game usually has no such env vars** — they are a developer
    /// shell convention, not persisted at user or machine level. The frontend
    /// resolves paths itself and calls [`load_from`](Self::load_from); this
    /// remains for headless tests and shell-launched runs.
    #[func]
    fn load(&mut self) -> bool {
        self.finish_load(MissionCore::load())
    }

    /// Load the DE kernels from two explicit filesystem paths (absolute, or
    /// relative to the process CWD — *not* `res://` paths; globalize them first).
    /// Returns `true` on success; `false` + [`last_error`](Self::last_error)
    /// otherwise. This is the frontend's entry point.
    #[func]
    fn load_from(&mut self, bsp_path: GString, pca_path: GString) -> bool {
        let r = MissionCore::load_from(&bsp_path.to_string(), &pca_path.to_string());
        self.finish_load(r)
    }

    /// Arm the small-body kernel (`sb441-n16.bsp`) at an explicit path. Returns
    /// `true`, or `false` + [`last_error`](Self::last_error) if the path is not a
    /// file. Call it after `load_from` and **before** `begin_build_scenario` — the
    /// mount happens on the build worker.
    ///
    /// Nothing is read here and nothing is slow here: this records a path. The
    /// asteroids appear when the build lands, not when this returns.
    #[func]
    fn set_small_body_kernel(&mut self, path: GString) -> bool {
        let Some(core) = self.core.as_mut() else {
            self.error = "load() must succeed before set_small_body_kernel()".into();
            return false;
        };
        match core.set_small_body_kernel(&path.to_string()) {
            Ok(()) => {
                self.error = GString::new();
                true
            }
            Err(e) => {
                self.error = GString::from(&e.to_string());
                false
            }
        }
    }

    /// Whether the served almanac actually has the small-body kernel mounted.
    ///
    /// **Gate every asteroid draw on this.** False means every small-body lookup
    /// fails, and a failed lookup that reaches the display is not a blank — it is a
    /// body sitting exactly on the Sun. This project has shipped that bug three
    /// times; the flag is cheaper than the fourth.
    #[func]
    fn small_bodies_mounted(&self) -> bool {
        self.core
            .as_ref()
            .is_some_and(|c| c.small_bodies_mounted())
    }

    /// How many small bodies the mounted kernel offers — `0` when it is not
    /// mounted, so a caller that ignores
    /// [`small_bodies_mounted`](Self::small_bodies_mounted) still iterates nothing
    /// rather than sixteen bodies that all resolve to the Sun.
    #[func]
    fn small_body_count(&self) -> i64 {
        if self.small_bodies_mounted() {
            SB441_BODIES.len() as i64
        } else {
            0
        }
    }

    /// The NAIF id of small body `i`, or `0` if out of range / not mounted. Feed it
    /// straight to [`body_position_ecl_au`](Self::body_position_ecl_au) — asteroids
    /// travel the same ephemeris read path as the planets, which is the whole point
    /// of mounting a kernel instead of integrating elements.
    #[func]
    fn small_body_id(&self, i: i64) -> i64 {
        if !self.small_bodies_mounted() {
            return 0;
        }
        usize::try_from(i)
            .ok()
            .and_then(|i| SB441_BODIES.get(i))
            .map_or(0, |(id, _)| *id as i64)
    }

    /// The name of small body `i`, or `""` if out of range / not mounted.
    #[func]
    fn small_body_name(&self, i: i64) -> GString {
        if !self.small_bodies_mounted() {
            return GString::new();
        }
        usize::try_from(i)
            .ok()
            .and_then(|i| SB441_BODIES.get(i))
            .map_or_else(GString::new, |(_, n)| GString::from(*n))
    }

    /// The kernel's usable coverage window as `[lo, hi]` seconds past J2000 — an
    /// **empty** array if not loaded. Discovered from the mounted kernel, not
    /// hardcoded (de440s ≈ 1850–2149, de441 ≈ 1550–2650), so the frontend clamps
    /// its clock to real coverage. f64 rather than a `Vector2` because a TDB
    /// second near 1e9 would lose ~64 s as f32.
    ///
    /// Clamping to this is not cosmetic: outside coverage every body lookup fails,
    /// and a failed lookup returns `Vector3::ZERO` — which in this heliocentric
    /// frame *is the Sun's position*. An unclamped clock does not blank the
    /// display, it silently collapses every planet onto the Sun.
    #[func]
    fn usable_span_tdb(&self) -> PackedFloat64Array {
        let mut arr = PackedFloat64Array::new();
        if let Some((lo, hi)) = self.core.as_ref().map(|c| c.usable_span_tdb()) {
            arr.push(lo);
            arr.push(hi);
        }
        arr
    }

    /// The span the threat exists over — `[start, end]` seconds past J2000, or an
    /// **empty** array before the scenario is built.
    ///
    /// The display must hide the threat outside this window, for exactly the
    /// reason [`usable_span_tdb`](Self::usable_span_tdb) exists: outside it every
    /// threat lookup fails, and a failed lookup is `Vector3::ZERO` — the Sun. The
    /// clock clamp does not cover this. It is clamped to the *kernel* (~300 years);
    /// the threat is propagated over ~12, so the great majority of the scrub range
    /// is outside it.
    #[func]
    fn threat_span_tdb(&self) -> PackedFloat64Array {
        let mut arr = PackedFloat64Array::new();
        if let Some((lo, hi)) = self.core.as_ref().and_then(|c| c.threat_span_tdb()) {
            arr.push(lo);
            arr.push(hi);
        }
        arr
    }

    /// Shared tail of [`load`](Self::load) / [`load_from`](Self::load_from): adopt
    /// the core on success, or drop it and record why on failure. Kept in one
    /// place so both entry points cannot drift on the error contract — a failed
    /// load must always leave `core` empty, never a stale one from a prior load.
    fn finish_load(&mut self, result: Result<MissionCore, ScenarioError>) -> bool {
        match result {
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

    /// Start building the designer impactor + campaign **on a worker thread**, and
    /// return immediately. Returns `true` if a build was started; `false` +
    /// [`last_error`](Self::last_error) if one is already in flight or the kernels
    /// are not loaded. Drive it with [`poll_build`](Self::poll_build).
    ///
    /// There is deliberately **no blocking form of this**. The build is ~10 s of
    /// integration, so calling it inline would freeze Godot's main thread — and the
    /// display it would freeze is a *working* one, since the orrery has been drawing
    /// real planets from the fast `load()` since 3C-2a. A synchronous entry point
    /// here would exist only to be misused.
    ///
    /// The worker gets a clone of the `Arc<Ephemeris>`, not this object: the core
    /// stays here answering `body_position_ecl_au` every frame while the scenario
    /// builds behind it. Nothing about `Mission` (a `RefCounted`) crosses the
    /// thread boundary — only a plain `Arc` out and a `BuiltScenario` back.
    #[func]
    fn begin_build_scenario(&mut self) -> bool {
        if self.build.is_some() {
            self.error = "a scenario build is already in flight".into();
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_build_scenario()".into();
            return false;
        };
        let served = core.ephemeris_arc();
        let (bsp, pca, small_bodies) = core.kernel_paths();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            // Mount the small-body kernel if one was armed — ~5.7 s cold on 646 MB,
            // which is why it happens here and not on the load path. The result is a
            // *second* almanac: `with_constants` consumes `self`, and the one that
            // came out of `ephemeris_arc` is being read by the renderer every frame.
            //
            // A mount failure is not fatal. The mission is a complete, correct
            // mission without asteroids; taking the whole build down over the
            // scenery would trade a missing catalog for a missing threat.
            let (eph, mounted) = match small_bodies.as_deref() {
                Some(sb) => match mount_small_bodies(&bsp, &pca, sb) {
                    Ok(e) => (Arc::new(e), true),
                    Err(e) => {
                        godot_warn!("small-body kernel not mounted, catalog will be empty: {e}");
                        (Arc::clone(&served), false)
                    }
                },
                None => (Arc::clone(&served), false),
            };
            // The error is flattened to a String on this side of the channel: only
            // the message ever reaches the HUD, and a plain String is unambiguously
            // safe to send.
            let result = BuiltScenario::build(
                Arc::clone(&eph),
                &ImpactorConfig::default(),
                mounted,
            )
            .map_err(|e| e.to_string())
                .and_then(|built| {
                    // The orrery's scenery flies here, on this thread, in the field
                    // that was just built — ~4 s of integration that would otherwise
                    // land on the main thread, since `add_synthetic_body` is
                    // inline-and-expensive by design.
                    let comet = seed_orrery_body(
                        &eph,
                        built.scenario_ref(),
                        display_comet::NAME,
                        display_comet::KIND,
                        display_comet::elements(),
                        built.epoch0(),
                        display_comet::CADENCE_SECONDS,
                        display_comet::N_SNAPSHOTS,
                    )
                    .map_err(|e| e.to_string())?;

                    // The real asteroids join the same catalog — but they cost no
                    // integration at all. A `.neo` table already holds JPL's
                    // trajectory, so this is a file read (milliseconds) beside the
                    // comet's ~4 s of flying. It rides the worker because this is
                    // where the catalog is assembled, not because it is expensive.
                    //
                    // Absent tables are the ordinary state of a fresh clone and
                    // produce an empty vector, exactly as an unmounted small-body
                    // kernel produces an empty asteroid list.
                    let mut bodies = vec![comet];
                    bodies.extend(load_neo_bodies());
                    Ok((built, bodies))
                });
            // A closed channel means the game quit mid-build. Dropping the result is
            // the right response; `send`'s Err must not become a panic on a detached
            // thread.
            let _ = tx.send(result);
        });
        self.build = Some(rx);
        self.error = GString::new();
        true
    }

    /// Whether a background scenario build is currently in flight.
    #[func]
    fn is_building(&self) -> bool {
        self.build.is_some()
    }

    /// Pump the background build: install the scenario if it has landed. Returns
    /// `true` while the build is **still running**, `false` once it is finished —
    /// at which point [`is_ready`](Self::is_ready) says whether it succeeded and
    /// [`last_error`](Self::last_error) says why if it did not.
    ///
    /// Non-blocking, so it is safe to call every frame. Cheap: a `try_recv` on an
    /// empty channel.
    #[func]
    fn poll_build(&mut self) -> bool {
        let Some(rx) = self.build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok((built, bodies))) => {
                self.build = None;
                match self.core.as_mut() {
                    Some(core) => {
                        core.install(built, bodies);
                        self.error = GString::new();
                    }
                    // The kernels were dropped (a failed re-load) while the build
                    // ran, so there is nothing to install it into. Say so rather
                    // than discard it silently and read as "still not ready".
                    None => {
                        self.error =
                            "the scenario finished building but the kernels are no longer loaded"
                                .into()
                    }
                }
                false
            }
            Ok(Err(message)) => {
                self.build = None;
                self.error = message.as_str().into();
                false
            }
            // The worker panicked and took the sender with it. A build that dies
            // without a word must not leave the frontend polling forever.
            Err(mpsc::TryRecvError::Disconnected) => {
                self.build = None;
                self.error = "the scenario build thread died without reporting".into();
                false
            }
        }
    }

    /// The nominal encounter's focused capture radius `b_capture`, m (`-1.0` if no
    /// scenario) — the radius of Earth's effective collision disc in the b-plane,
    /// ~1.773 R⊕ at this encounter's `v_inf`.
    ///
    /// The bar a deflection verdict is measured against, and it measures the
    /// **impact parameter** — [`deflected_impact_parameter_m`](
    /// Self::deflected_impact_parameter_m), not the perigee. `b > b_capture` is the
    /// core's own [`is_hit`] criterion, which it proves equivalent to `perigee >
    /// R⊕`. The two are equivalent only *as pairs*: b is the un-focused asymptotic
    /// miss and b_capture is the target enlarged to account for focusing, while the
    /// perigee already *is* the focused closest approach and so belongs against
    /// Earth's solid radius. Comparing a perigee against this number mixes the pairs
    /// and silently demands ~1.5× more miss than physics does. (It is also not
    /// [`is_clean_miss`](Self::is_clean_miss), a far wider bar a safe plan need not
    /// reach.)
    ///
    /// [`is_hit`]: asteroid_core::geometry::BPlaneEncounter::is_hit
    #[func]
    fn capture_radius_m(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.capture_radius_m())
            .unwrap_or(-1.0)
    }

    /// The nominal (un-deflected) b-plane perigee, m (`-1.0` if no scenario) — how
    /// close the incoming rock actually comes to Earth's centre. Inside R⊕ by
    /// construction: it is a surface impact.
    #[func]
    fn nominal_perigee_m(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.nominal_perigee_m())
            .unwrap_or(-1.0)
    }

    /// The nominal pass's b-plane impact parameter `b`, m (`-1.0` if no scenario) —
    /// the hit being undone, inside [`capture_radius_m`](Self::capture_radius_m) by
    /// construction.
    #[func]
    fn nominal_impact_parameter_m(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.nominal_impact_parameter_m())
            .unwrap_or(-1.0)
    }

    /// Earth's solid-body radius `R⊕` as the core models it, m (`-1.0` if no
    /// scenario) — the disc to draw, and the bar a *perigee* is measured against.
    #[func]
    fn earth_radius_m(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.earth_radius_m())
            .unwrap_or(-1.0)
    }

    /// The nominal encounter's hyperbolic excess speed `v_inf`, m/s (`-1.0` if no
    /// scenario) — the approach speed "at infinity" that sets the focusing.
    ///
    /// Not the config's 18 km/s `v_rel`, which is the speed at the 3000 km impact
    /// point deep in Earth's well; with the well stripped out this is ~7.63 km/s,
    /// and that is what makes the capture disc 1.773 R⊕ rather than ~1.18.
    #[func]
    fn encounter_v_inf_m_s(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.encounter_v_inf_m_s())
            .unwrap_or(-1.0)
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

    /// The designer campaign's impact epoch, seconds past J2000 — **without**
    /// building the scenario, and available before [`load`](Self::load).
    ///
    /// This is knowable cheaply because the impact epoch is a config *input*
    /// (`ImpactorConfig::default()`), not something the build solves for: the
    /// designer says when the rock arrives and the builder works backward to a
    /// seed. So the frontend can anchor its clock on the real campaign timeline
    /// without paying the multi-year back-propagation, and the real threat later
    /// drops onto an already-correct timeline.
    #[func]
    fn default_impact_tdb_seconds(&self) -> f64 {
        ImpactorConfig::default()
            .impact_epoch
            .tdb_seconds_past_j2000()
    }

    /// The designer campaign's start epoch (`impact − lead_years`), seconds past
    /// J2000 — same cheap, pre-build contract as
    /// [`default_impact_tdb_seconds`](Self::default_impact_tdb_seconds), and
    /// derived through the same `ImpactorConfig::epoch0` the builder itself uses,
    /// so the drawn campaign cannot drift from the built one.
    #[func]
    fn default_epoch0_tdb_seconds(&self) -> f64 {
        ImpactorConfig::default().epoch0().tdb_seconds_past_j2000()
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

    /// The deflected pass's b-plane impact parameter `b`, m. `-1.0` if no plan is
    /// set **or** the pass is a clean miss — distinguish those with
    /// [`has_plan`](Self::has_plan) / [`is_clean_miss`](Self::is_clean_miss).
    ///
    /// **The miss the verdict is made of.** Safe is `b > capture_radius_m()`; this
    /// is the number to print beside that one, because those two are the pair the
    /// core's hit test compares. See [`capture_radius_m`](Self::capture_radius_m).
    #[func]
    fn deflected_impact_parameter_m(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.deflected_impact_parameter_m())
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

    // --- the b-plane encounter view (3C-2c) ---------------------------------
    //
    // The encounter arrives already projected into the core-derived b-plane display
    // basis, as `(ξ, ζ, s)` **kilometres**: ξ/ζ are the in-plane axes to draw, and s
    // is depth along the incoming asymptote (negative inbound, positive outbound) so
    // the view can shade the approach without owning any geometry.
    //
    // f32 is safe here despite the tracks reaching ~10⁷ km at the window edge: the
    // core subtracted Earth's position in f64 and only this geocentric residual
    // crosses, so the error scales with the value (~1 km out at the edge, millimetres
    // at the ~10⁴ km perigee that actually decides anything) — HANDOFF §7.

    /// The nominal (impact) encounter track — `ENCOUNTER_SAMPLES` `(ξ, ζ, s)` km
    /// points across the ±1.5 d window. Empty before the scenario is built.
    ///
    /// Available with **no plan**: this is the incoming impact, and it is the whole
    /// picture until the player does something about it. Cache it — it never changes.
    #[func]
    fn encounter_nominal_track_km(&self) -> PackedVector3Array {
        Self::pack(
            self.core
                .as_ref()
                .map(|c| c.encounter_nominal_track_km())
                .unwrap_or_default(),
        )
    }

    /// The deflected encounter track in the same frame — `(ξ, ζ, s)` km points.
    /// **Empty until a plan is solved**, which is not a zero-length track: draw
    /// nothing, since a zeroed one would run the asteroid through Earth's centre.
    /// Re-read after [`set_plan`](Self::set_plan).
    #[func]
    fn encounter_deflected_track_km(&self) -> PackedVector3Array {
        Self::pack(
            self.core
                .as_ref()
                .map(|c| c.encounter_deflected_track_km())
                .unwrap_or_default(),
        )
    }

    /// The encounter tracks' sample epochs as `[first, last]` seconds past J2000, or
    /// an **empty** array before the scenario is built. Samples are uniformly spaced
    /// and shared by both tracks, so a clock time maps to a track index directly.
    #[func]
    fn encounter_sample_span_tdb(&self) -> PackedFloat64Array {
        let mut arr = PackedFloat64Array::new();
        if let Some((lo, hi)) = self.core.as_ref().and_then(|c| c.encounter_sample_span_tdb()) {
            arr.push(lo);
            arr.push(hi);
        }
        arr
    }

    /// Where the **nominal** incoming asymptote pierces the b-plane — `(ξ, ζ, s)`
    /// km, `Vector3::ZERO` before the scenario is built. Its distance from the
    /// origin is [`nominal_impact_parameter_m`](Self::nominal_impact_parameter_m),
    /// and it lies inside the capture disc: the hit.
    ///
    /// The core leaves the b-vector's *sign* unpinned (a Tier-3 keyhole question),
    /// so which side of the disc this lands on is cosmetic. Its **distance** is not.
    #[func]
    fn nominal_b_point_km(&self) -> Vector3 {
        Self::to_v3(self.core.as_ref().and_then(|c| c.nominal_b_point_km()))
    }

    /// Where the **deflected** asymptote pierces the b-plane — `(ξ, ζ, s)` km.
    /// `Vector3::ZERO` if no plan or a clean miss (no finite b-plane point exists
    /// once the pass has left the scan gate) — and ZERO is Earth's centre here, so
    /// gate on [`has_plan`](Self::has_plan) / [`is_clean_miss`](Self::is_clean_miss)
    /// rather than drawing it unconditionally.
    #[func]
    fn deflected_b_point_km(&self) -> Vector3 {
        Self::to_v3(self.core.as_ref().and_then(|c| c.deflected_b_point_km()))
    }

    /// f64 nalgebra points → a Godot `PackedVector3Array` (the f32 cast at the FFI
    /// boundary, in one place).
    fn pack(pts: Vec<nalgebra::Vector3<f64>>) -> PackedVector3Array {
        let mut arr = PackedVector3Array::new();
        for v in pts {
            arr.push(Vector3::new(v.x as f32, v.y as f32, v.z as f32));
        }
        arr
    }

    /// An optional f64 nalgebra vector → a Godot `Vector3`, absent becoming ZERO.
    fn to_v3(v: Option<nalgebra::Vector3<f64>>) -> Vector3 {
        match v {
            Some(v) => Vector3::new(v.x as f32, v.y as f32, v.z as f32),
            None => Vector3::ZERO,
        }
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

    /// Where catalog body `index`'s positions come from: `"integrated"` (this
    /// project's physics, flown in the validated Tier-1 field) or `"sampled"`
    /// (JPL's, read from a Horizons state table and interpolated). Empty if out
    /// of range.
    ///
    /// The frontend labels bodies with this. Drawing someone else's trajectory
    /// beside our own with nothing distinguishing them is precisely the mistake
    /// the GDScript Kepler propagator was.
    #[func]
    fn catalog_provenance(&self, index: i64) -> GString {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.core.as_ref().and_then(|c| c.catalog_provenance(i)))
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

    /// Catalog body `index`'s orbit as `samples` points across `[t0_tdb, t1_tdb]` —
    /// the polyline over an arbitrary window, so a decades-long sampled body draws
    /// one lap instead of dozens overplotted. Points outside the body's span are
    /// dropped rather than drawn at the Sun. Empty if the index is invalid.
    #[func]
    fn catalog_track_window_ecl_au(
        &self,
        index: i64,
        t0_tdb: f64,
        t1_tdb: f64,
        samples: i64,
    ) -> PackedVector3Array {
        let n = samples.max(0) as usize;
        let pts = usize::try_from(index)
            .ok()
            .and_then(|i| {
                self.core
                    .as_ref()
                    .map(|c| c.catalog_track_window_ecl_au(i, t0_tdb, t1_tdb, n))
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

    /// One orbital period of catalog body `index`, seconds — or the whole covered
    /// span where a period is not meaningful. `0.0` if the index is invalid.
    ///
    /// The orbit line samples this rather than the full span: a real NEO's table
    /// runs decades while its orbit is about a year, so the whole span is dozens
    /// of laps overplotted into noise.
    #[func]
    fn catalog_orbit_period_seconds(&self, index: i64) -> f64 {
        usize::try_from(index)
            .ok()
            .and_then(|i| {
                self.core
                    .as_ref()
                    .and_then(|c| c.catalog_orbit_period_seconds(i))
            })
            .unwrap_or(0.0)
    }
}
