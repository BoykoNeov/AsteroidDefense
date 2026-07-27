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
use asteroid_core::launch_vehicle::LaunchVehicle;
use mission_core::{
    display_comet, launch_vehicle, launch_vehicle_count, load_neo_bodies, measure_tier2_shifts,
    mount_small_bodies, seed_orrery_body, verify_porkchop_cell, BuiltScenario, CellVerdict,
    MissionCore, OrreryBody, PorkchopView, Tier2Shifts, SB441_BODIES,
};

/// The launcher at a GDScript-supplied index, or `None` for a negative or
/// out-of-range one. A free function so every `#[func]` that takes a `vehicle`
/// argument resolves it exactly one way.
fn vehicle_at(index: i64) -> Option<&'static LaunchVehicle> {
    if index < 0 {
        return None;
    }
    launch_vehicle(index as usize)
}

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
    /// The in-flight on-demand Tier-2 shift measurement, if any — see
    /// [`begin_tier2_preview`](Mission::begin_tier2_preview). `Some` exactly while a
    /// preview worker is running; independent of `build` (a scenario is fully usable
    /// without ever measuring the menu).
    tier2_build: Option<mpsc::Receiver<Result<Tier2Shifts, String>>>,
    /// The in-flight porkchop grid build, if any — see
    /// [`begin_porkchop`](Mission::begin_porkchop). A **third** independent channel:
    /// the grid, the Tier-2 preview and the scenario build are unrelated pieces of
    /// work and none of them should be able to block or cancel another.
    porkchop_build: Option<mpsc::Receiver<Result<PorkchopView, String>>>,
    /// The built grid. Lives here rather than in [`MissionCore`] because it is a
    /// *display artifact* — a projection of the scenario for one view — not part of
    /// the mission state the core owns. Dropped whenever a new scenario is installed
    /// (see [`poll_build`](Mission::poll_build)).
    porkchop: Option<PorkchopView>,
    /// The in-flight on-demand full-field verify of one selected cell — its own
    /// channel again, because a verify is fired repeatedly against a grid that stays
    /// put, and must not disturb it.
    verify_build: Option<mpsc::Receiver<Result<CellVerdict, String>>>,
    /// Which cell the in-flight verify is for — `(launch, arrival, vehicle,
    /// impactor kg)`. Held here rather than sent through the channel because the
    /// worker computes physics, not identity, and pairing them on arrival keeps the
    /// verdict from ever being labelled with a cell it did not come from.
    pending_verify: (i64, i64, i64, f64),
    /// The last cell verdict and which cell it belongs to, so the display can tell
    /// "this cursor's verdict" from "a verdict for a cell I have since left".
    verdict: Option<(i64, i64, i64, f64, CellVerdict)>,
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
            // The Tier-2 shift preview is DELIBERATELY not measured here: it is ~64 s
            // of propagation that would sit *before* `install`, delaying the threat
            // solution and the planner — the core gameplay — by that much. It is
            // instead computed on demand when the operator opens the force-model menu
            // (`begin_tier2_preview`), off the same scenario, so the threat lands as
            // fast as it did before the menu existed.
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
                        // A porkchop belongs to the scenario it was solved against —
                        // its axes come from that campaign's epochs and its cells
                        // from that nominal trajectory. Installing a new scenario
                        // makes the old grid a picture of a threat that is no longer
                        // there, so it is dropped rather than left to be read.
                        self.porkchop = None;
                        self.verdict = None;
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

    /// Kick off the on-demand Tier-2 shift measurement on a worker thread — the
    /// ~64 s the force-model menu costs, paid only when the operator opens it and
    /// **off the critical build path** so the threat solution is never delayed.
    ///
    /// The worker gets an `Arc` clone of the built scenario (a refcount bump, not a
    /// rebuild — the shifts come off the *exact* scenario the threat was flown in),
    /// measures the five single-term shifts ([`measure_tier2_shifts`]) and sends them
    /// back for [`poll_tier2_preview`](Self::poll_tier2_preview) to adopt. Returns
    /// `false` (a no-op) if the preview is already measured or already in flight, or
    /// if there is no scenario to measure against yet.
    #[func]
    fn begin_tier2_preview(&mut self) -> bool {
        if self.tier2_build.is_some() {
            return false; // already measuring — not an error, just nothing new to do
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_tier2_preview()".into();
            return false;
        };
        if core.has_tier2_preview() {
            return false; // already measured; the numbers are cached
        }
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before measuring Tier-2 shifts".into();
            return false;
        };
        let mounted = core.small_bodies_mounted();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = measure_tier2_shifts(&scenario, mounted).map_err(|e| e.to_string());
            let _ = tx.send(result);
        });
        self.tier2_build = Some(rx);
        self.error = GString::new();
        true
    }

    /// Whether the on-demand Tier-2 shift measurement is currently in flight.
    #[func]
    fn is_measuring_tier2(&self) -> bool {
        self.tier2_build.is_some()
    }

    /// Pump the Tier-2 preview worker: adopt the shifts if they have landed. Returns
    /// `true` while the measurement is **still running**, `false` once it is finished
    /// (or none is in flight) — at which point [`has_tier2_preview`](Self::has_tier2_preview)
    /// says whether it succeeded. Non-blocking; safe to call every frame.
    #[func]
    fn poll_tier2_preview(&mut self) -> bool {
        let Some(rx) = self.tier2_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(shifts)) => {
                self.tier2_build = None;
                match self.core.as_mut() {
                    Some(core) => {
                        core.adopt_tier2_shifts(shifts);
                        self.error = GString::new();
                    }
                    None => {
                        self.error =
                            "the Tier-2 preview finished but the mission is no longer loaded".into()
                    }
                }
                false
            }
            Ok(Err(message)) => {
                self.tier2_build = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.tier2_build = None;
                self.error = "the Tier-2 preview thread died without reporting".into();
                false
            }
        }
    }

    // --- The porkchop grid (HANDOFF §8) -------------------------------------

    /// Kick off the launch × arrival porkchop grid on a worker thread — the cheap,
    /// **vehicle-independent** half of the deliverability layer.
    ///
    /// Both axes are derived from the built scenario's own campaign, so this needs
    /// a scenario, not merely kernels. Measured cost is ~45 µs/cell (each cell
    /// selects the cheapest transfer across the direct arc and both branches of one
    /// lapping alternative), so a 120×120 grid is ~0.6 s — off-thread, once, and
    /// never per frame. Returns `false` if a grid is already in flight or there is
    /// nothing to build against.
    ///
    /// Rebuilding is allowed: calling this with different sample counts replaces the
    /// grid when the new one lands.
    #[func]
    fn begin_porkchop(&mut self, launch_samples: i64, arrival_samples: i64) -> bool {
        if self.porkchop_build.is_some() {
            return false; // already building — not an error
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_porkchop()".into();
            return false;
        };
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before building the porkchop".into();
            return false;
        };
        let (nl, na) = (
            launch_samples.clamp(2, 512) as usize,
            arrival_samples.clamp(2, 512) as usize,
        );
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = PorkchopView::build(&scenario, nl, na).map_err(|e| e.to_string());
            let _ = tx.send(result);
        });
        self.porkchop_build = Some(rx);
        self.error = GString::new();
        true
    }

    /// Whether the porkchop grid is currently being built.
    #[func]
    fn is_building_porkchop(&self) -> bool {
        self.porkchop_build.is_some()
    }

    /// Pump the porkchop worker. `true` while **still running**, `false` once
    /// finished (or none in flight) — then [`has_porkchop`](Self::has_porkchop) says
    /// whether it succeeded. Non-blocking; safe every frame.
    #[func]
    fn poll_porkchop(&mut self) -> bool {
        let Some(rx) = self.porkchop_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(view)) => {
                self.porkchop_build = None;
                self.porkchop = Some(view);
                // A verdict describes a cell of the *previous* grid; the new grid's
                // axes may differ, so the same (i, j) is a different window.
                self.verdict = None;
                self.error = GString::new();
                false
            }
            Ok(Err(message)) => {
                self.porkchop_build = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.porkchop_build = None;
                self.error = "the porkchop build thread died without reporting".into();
                false
            }
        }
    }

    /// Whether a built grid is available to read.
    #[func]
    fn has_porkchop(&self) -> bool {
        self.porkchop.is_some()
    }

    /// Rows in the grid (launch epochs); `0` if none is built.
    #[func]
    fn porkchop_launch_count(&self) -> i64 {
        self.porkchop.as_ref().map_or(0, |p| p.launch_count() as i64)
    }

    /// Columns in the grid (arrival epochs); `0` if none is built.
    #[func]
    fn porkchop_arrival_count(&self) -> i64 {
        self.porkchop.as_ref().map_or(0, |p| p.arrival_count() as i64)
    }

    /// The launch axis, TDB seconds past J2000.
    #[func]
    fn porkchop_launch_tdb(&self) -> PackedFloat64Array {
        self.porkchop
            .as_ref()
            .map(|p| PackedFloat64Array::from(p.launch_tdb().as_slice()))
            .unwrap_or_default()
    }

    /// The arrival axis, TDB seconds past J2000.
    #[func]
    fn porkchop_arrival_tdb(&self) -> PackedFloat64Array {
        self.porkchop
            .as_ref()
            .map(|p| PackedFloat64Array::from(p.arrival_tdb().as_slice()))
            .unwrap_or_default()
    }

    /// Departure `C3` per cell, km²/s², row-major `[launch][arrival]`.
    ///
    /// **`-1.0` marks a cell with no transfer at any allowed revolution count.** A
    /// negative `C3` is physically impossible, so the sentinel is unambiguous — and
    /// deliberately not `NaN`, which would poison every min/max the heatmap
    /// normalizes by and flatten the whole picture to one colour.
    ///
    /// This is the display's **only** authority on emptiness. The other columns
    /// carry ordinary zeros in blank cells, so reading them for emptiness would
    /// confuse "no trajectory exists" with "a trajectory that projects to nothing" —
    /// and a third state, "a real transfer this launcher cannot reach"
    /// ([`porkchop_payload_kg`](Self::porkchop_payload_kg) `== 0`), must stay
    /// distinct from both.
    #[func]
    fn porkchop_c3(&self) -> PackedFloat64Array {
        self.pork_col(|p| p.c3_flat())
    }

    /// Signed along-track projection per cell, m/s (`0` in blank cells). Negative is
    /// a retrograde, orbit-shrinking push — a real lever, not bad aim.
    #[func]
    fn porkchop_along_track(&self) -> PackedFloat64Array {
        self.pork_col(|p| p.along_track_flat())
    }

    /// Arrival relative speed per cell, m/s (`0` in blank cells).
    #[func]
    fn porkchop_arrival_v_rel(&self) -> PackedFloat64Array {
        self.pork_col(|p| p.arrival_v_rel_flat())
    }

    /// Complete laps of the Sun per cell; `-1` in blank cells. `0` is the direct
    /// arc — anything higher is a genuinely different cruise, not just a different
    /// number, which is why the grid reports it.
    #[func]
    fn porkchop_revolutions(&self) -> PackedInt32Array {
        self.porkchop
            .as_ref()
            .map(|p| PackedInt32Array::from(p.revolutions_flat().as_slice()))
            .unwrap_or_default()
    }

    /// Deliverable impactor mass per cell for launcher `vehicle`, kg.
    ///
    /// `0` means **this launcher cannot reach that `C3`** — and is *also* `0` where
    /// no transfer exists at all. Read against [`porkchop_c3`](Self::porkchop_c3) to
    /// separate them; they are different facts and a display that draws them the
    /// same way throws away the point of a vehicle-independent grid.
    #[func]
    fn porkchop_payload_kg(&self, vehicle: i64) -> PackedFloat64Array {
        match (self.porkchop.as_ref(), vehicle_at(vehicle)) {
            (Some(p), Some(v)) => PackedFloat64Array::from(p.payload_kg_flat(v).as_slice()),
            _ => PackedFloat64Array::new(),
        }
    }

    /// The along-track Δv the delivered mass imparts per cell, m/s (signed; `0`
    /// where the launcher cannot reach the cell, or no transfer exists).
    #[func]
    fn porkchop_along_track_dv(&self, vehicle: i64) -> PackedFloat64Array {
        match (self.porkchop.as_ref(), vehicle_at(vehicle)) {
            (Some(p), Some(v)) => PackedFloat64Array::from(p.along_track_dv_flat(v).as_slice()),
            _ => PackedFloat64Array::new(),
        }
    }

    /// Everything the readout shows for one cell, in **one** call so a row can never
    /// be assembled out of two different cells. An **empty dictionary** means the
    /// indices are out of range or the cell holds no transfer — never a zero-filled
    /// row, which would read as a real but useless window.
    #[func]
    fn porkchop_cell(&self, i: i64, j: i64, vehicle: i64) -> VarDictionary {
        let mut d = VarDictionary::new();
        let (Some(p), Some(v)) = (self.porkchop.as_ref(), vehicle_at(vehicle)) else {
            return d;
        };
        if i < 0 || j < 0 {
            return d;
        }
        let Some(c) = p.detail(i as usize, j as usize, v) else {
            return d;
        };
        d.set("launch_tdb", c.launch_tdb);
        d.set("arrival_tdb", c.arrival_tdb);
        d.set("tof_days", c.tof_days);
        d.set("c3_km2_s2", c.c3_km2_s2);
        d.set("arrival_v_rel_ms", c.arrival_v_rel_ms);
        d.set("along_track_proj_ms", c.along_track_proj_ms);
        d.set("revolutions", c.revolutions as i64);
        d.set("payload_kg", c.payload_kg);
        d.set("along_track_dv_ms", c.along_track_dv_ms);
        d
    }

    /// How many launchers the frontend can cycle through — the core's own
    /// canonical table, so the display cannot offer a vehicle the physics does not
    /// have (or miss one it does).
    #[func]
    fn vehicle_count(&self) -> i64 {
        launch_vehicle_count() as i64
    }

    /// Launcher `i`'s name, or `""` past the end.
    #[func]
    fn vehicle_name(&self, i: i64) -> GString {
        vehicle_at(i).map_or_else(GString::new, |v| v.name.into())
    }

    /// The highest `C3` launcher `i` is tabulated for, km²/s² (`-1.0` past the end)
    /// — the launch energy above which it delivers nothing. The heatmap's natural
    /// upper colour bound for a vehicle-relative view.
    #[func]
    fn vehicle_max_c3(&self, i: i64) -> f64 {
        vehicle_at(i).map_or(-1.0, |v| v.max_c3_km2_s2())
    }

    // --- The on-demand full-field verify of one cell ------------------------

    /// Re-fly the asteroid through the **full `n`-body field** after the impulse
    /// this cell's window would actually deliver, on a worker thread.
    ///
    /// The impactor mass is the selected launcher's deliverable payload at that
    /// cell's `C3`, which is what makes this the honest question — not "would some
    /// impulse work" but "does *this launcher*, through *this window*, work". One
    /// propagation (~1 s), so it is fired per selected cell and never across a grid.
    ///
    /// Returns `false` — with a reason in [`last_error`](Self::last_error) — when a
    /// verify is already in flight, there is no grid, the indices are out of range,
    /// or the cell carries no transfer for this launcher to fly.
    #[func]
    fn begin_cell_verify(&mut self, i: i64, j: i64, vehicle: i64) -> bool {
        if self.verify_build.is_some() {
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_cell_verify()".into();
            return false;
        };
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before verifying a cell".into();
            return false;
        };
        let (Some(p), Some(v)) = (self.porkchop.as_ref(), vehicle_at(vehicle)) else {
            self.error = "no porkchop grid (or unknown launcher) to verify against".into();
            return false;
        };
        if i < 0 || j < 0 {
            self.error = "cell indices must be non-negative".into();
            return false;
        }
        let (Some(metrics), Some(detail)) = (
            p.metrics_at(i as usize, j as usize),
            p.detail(i as usize, j as usize, v),
        ) else {
            self.error = "that cell carries no transfer to verify".into();
            return false;
        };
        // A launcher that delivers nothing here has no mission to verify. Saying so
        // is the honest answer; running the propagation anyway would spend a second
        // to reproduce the nominal hit and print it as a *result*.
        if detail.payload_kg <= 0.0 {
            self.error = "this launcher delivers no mass at that cell's C3".into();
            return false;
        }
        let arrival_tdb = detail.arrival_tdb;
        let mass = detail.payload_kg;
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = verify_porkchop_cell(&scenario, arrival_tdb, &metrics, mass)
                .map_err(|e| e.to_string());
            let _ = tx.send(result);
        });
        self.verify_build = Some(rx);
        // Remember *which* cell this verify is for, and drop any previous verdict:
        // a stale verdict shown beside a running verify would read as the answer.
        self.pending_verify = (i, j, vehicle, mass);
        self.verdict = None;
        self.error = GString::new();
        true
    }

    /// Whether the on-demand cell verify is in flight.
    #[func]
    fn is_verifying_cell(&self) -> bool {
        self.verify_build.is_some()
    }

    /// Pump the verify worker. `true` while **still running**, `false` once finished
    /// (or none in flight) — then [`cell_verdict`](Self::cell_verdict) holds the
    /// result. Non-blocking; safe every frame.
    #[func]
    fn poll_cell_verify(&mut self) -> bool {
        let Some(rx) = self.verify_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(verdict)) => {
                self.verify_build = None;
                let (i, j, v, mass) = self.pending_verify;
                self.verdict = Some((i, j, v, mass, verdict));
                self.error = GString::new();
                false
            }
            Ok(Err(message)) => {
                self.verify_build = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.verify_build = None;
                self.error = "the cell verify thread died without reporting".into();
                false
            }
        }
    }

    /// The last full-field cell verdict, or an **empty dictionary** if none has been
    /// computed for the current grid.
    ///
    /// Keys always present: `launch_index`, `arrival_index`, `vehicle`,
    /// `impactor_kg` (which cell and which delivery this describes — so the display
    /// can tell "the cursor's verdict" from "a verdict for a cell I have left"), and
    /// `outcome`, one of:
    ///
    /// - `"clean_miss"` — the deflected pass left the close-approach scan gate
    ///   entirely. **The best possible result**, and it carries no b-plane numbers
    ///   because there is no encounter to reduce, not because they are missing. It
    ///   must never be collapsed onto the same sentinel as "not verified yet".
    /// - `"encounter"` — plus `impact_parameter_m`, `capture_radius_m`, `perigee_m`,
    ///   `earth_radius_m`, `is_hit`. **The verdict is `|B|` against
    ///   `capture_radius_m`** (which is exactly `is_hit`); the perigee pairs with
    ///   `earth_radius_m`. The two are equivalent only *as pairs* — comparing the
    ///   perigee against the capture radius is neither, and is silently ~1.5× too
    ///   strict.
    /// - `"not_hyperbolic"` — a dead-centre capture, a hit with no b-plane
    ///   reduction available.
    #[func]
    fn cell_verdict(&self) -> VarDictionary {
        let mut d = VarDictionary::new();
        let Some((i, j, v, mass, verdict)) = self.verdict else {
            return d;
        };
        d.set("launch_index", i);
        d.set("arrival_index", j);
        d.set("vehicle", v);
        d.set("impactor_kg", mass);
        match verdict {
            CellVerdict::CleanMiss => {
                d.set("outcome", "clean_miss");
            }
            CellVerdict::NotHyperbolic => {
                d.set("outcome", "not_hyperbolic");
            }
            CellVerdict::Encounter {
                impact_parameter_m,
                capture_radius_m,
                perigee_m,
                earth_radius_m,
                is_hit,
            } => {
                d.set("outcome", "encounter");
                d.set("impact_parameter_m", impact_parameter_m);
                d.set("capture_radius_m", capture_radius_m);
                d.set("perigee_m", perigee_m);
                d.set("earth_radius_m", earth_radius_m);
                d.set("is_hit", is_hit);
            }
        }
        d
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

    /// Whether the five single-term Tier-2 shifts were measured for this scenario
    /// — the frontend's cue that the GR/Yarkovsky/belt/SRP/J2 menu holds real numbers
    /// rather than the `-1` "not ready" sentinel. `true` only after a build that
    /// opted into the preview (the frontend's worker build does).
    #[func]
    fn has_tier2_preview(&self) -> bool {
        self.core.as_ref().is_some_and(|c| c.has_tier2_preview())
    }

    /// The **shifted nominal perigee**, m, that the fixed shipping seed reaches with
    /// a single Tier-2 term switched on — `term` ∈ {`"relativity"`, `"yarkovsky"`,
    /// `"belt"`, `"srp"`, `"j2"`}. `-1.0` when the preview was not run, `term` is
    /// unknown, or — **belt only** — the small-body kernel is absent, so the belt
    /// shift is genuinely *unavailable* rather than zero (a 0 would read as "the belt
    /// does nothing"). The GDScript menu forms the shift as
    /// `nominal_perigee_m() − this` and formats the km.
    #[func]
    fn tier2_shifted_perigee_m(&self, term: GString) -> f64 {
        self.core
            .as_ref()
            .and_then(|c| c.tier2_shifted_perigee_m(&term.to_string()))
            .unwrap_or(-1.0)
    }

    /// The `J2` perigee shift measured on a genuine **miss** geometry, km — the
    /// in-domain companion to the `"j2"` entry of
    /// [`tier2_shifted_perigee_m`](Self::tier2_shifted_perigee_m), which the panel's
    /// footnote cites.
    ///
    /// Every term in that menu is measured on the fixed shipping seed, and that seed
    /// is a designed **impact** whose closest approach is 3000 km — *inside* Earth,
    /// where the `J2` expansion is not valid. The menu number is what this geometry
    /// really does and it stays as measured; this is the number from the geometry
    /// that matters (a deflected pass, perigee outside `R_eq`), so the display can
    /// say which is which instead of printing one and implying the other.
    ///
    /// A recorded constant rather than a live measurement: it costs a full
    /// propagation pair and never changes for a given scenario. It is pinned to the
    /// core's own measurement by `earth_j2_on_a_deflected_miss_is_in_domain`, so this
    /// cannot silently drift from what the physics says — the treatment
    /// `SB441_BODIES` and `threat_mass_kg()` already get.
    #[func]
    fn j2_miss_geometry_shift_km(&self) -> f64 {
        asteroid_core::scenario::J2_DEFLECTED_MISS_PERIGEE_SHIFT_KM
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

    /// One vehicle-independent porkchop column → a `PackedFloat64Array`, empty when
    /// no grid is built. In one place so every column shares the same "no grid"
    /// answer — an empty array, which GDScript reads as `size() == 0` rather than as
    /// a grid of zeros.
    fn pork_col(&self, f: impl Fn(&PorkchopView) -> Vec<f64>) -> PackedFloat64Array {
        self.porkchop
            .as_ref()
            .map(|p| PackedFloat64Array::from(f(p).as_slice()))
            .unwrap_or_default()
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
