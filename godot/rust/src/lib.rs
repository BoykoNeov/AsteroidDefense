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

use asteroid_core::launch_vehicle::LaunchVehicle;
use asteroid_core::mission::MassSolveOutcome;
use asteroid_core::scenario::{ImpactorConfig, ScenarioError, SAFE_PERIGEE_TARGET_M};
use asteroid_core::{Epoch, OrbitalElements};
use mission_core::tractor_min_hover_radii;
use mission_core::{
    display_comet, heaviest_deliverable_kg, launch_vehicle, launch_vehicle_count, load_neo_bodies,
    measure_tier2_shifts, mount_small_bodies, probe_tow_plan, required_cell_mass, seed_orrery_body,
    solve_required_dv_anchor, tractor_readout as score_tractor_plan, verify_porkchop_cell,
    BuiltScenario, CellVerdict, KeyholePlanRow, MissionCore, OrreryBody, PorkchopView,
    ThreatOrbitKnobs, Tier2Shifts, Tier3View, TractorPlan, REQUIRED_DV_LAW_MIN_PERIODS,
    SB441_BODIES, THREAT_RADIUS_M, TRACTOR_HOVER_RADII,
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
    /// The in-flight kernel read, if any — see [`begin_load`](Mission::begin_load).
    /// A **ninth** channel, and the earliest one: it runs before there is a core at
    /// all, which is what makes it different from every other worker here.
    ///
    /// It exists because the read is not free on a spinning disk. Warm it is 27 ms
    /// and nobody would thread it; cold, the 32 MB `de440s.bsp` measured 2.4–6.4 s
    /// off this machine's drive (unbuffered, 5–13 MB/s — the same disk streams a
    /// 200 MB file at 35 MB/s, so the small kernel is seek-bound), and every
    /// millisecond of it used to be main-thread time before the window drew
    /// anything at all.
    field_load: Option<mpsc::Receiver<Result<MissionCore, String>>>,
    /// The in-flight background scenario build, if any — see
    /// [`begin_build_scenario`](Mission::begin_build_scenario). `Some` exactly while
    /// a worker is running, so it doubles as the "is building" flag.
    build: Option<mpsc::Receiver<Result<(BuiltScenario, Vec<OrreryBody>), String>>>,
    /// The display comet's ~4 s flight, if any — see
    /// [`poll_catalog`](Mission::poll_catalog). It rode the build worker until the
    /// phase split was measured: of `BuiltScenario::build`'s 11.8 s, 10.7 s is the
    /// forward nominal flight, so the comet is the one job with somewhere to hide.
    /// It now starts when the scenario is *installed* and lands into the catalog
    /// afterwards, which takes it off the wait for the threat solution entirely.
    comet_build: Option<mpsc::Receiver<Result<OrreryBody, String>>>,
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
    /// The in-flight required-impactor-mass solve — a **fifth** channel, for the
    /// same reason as the fourth: it is fired repeatedly against a grid that stays
    /// put and must not disturb the verify, the grid, or the build.
    mass_build: Option<mpsc::Receiver<Result<MassSolveOutcome, String>>>,
    /// The in-flight gravity-tractor probe — a **sixth** independent channel, on
    /// the same principle as the fourth and fifth: the tractor panel fires probes
    /// repeatedly against a scenario that stays put, and must not be able to
    /// disturb (or be disturbed by) the grid, the verify, the mass solve or the
    /// build.
    tow_build: Option<mpsc::Receiver<Result<(f64, f64), String>>>,
    /// The plan the in-flight probe is for. Held here, not sent through the
    /// channel, for the reason `pending_verify` documents: the worker computes
    /// physics, not identity, and pairing them on arrival is what stops a perigee
    /// being labelled with knobs it was not solved for.
    pending_tow: Option<TractorPlan>,
    /// The last probe result and the plan it belongs to — `(plan, towed perigee,
    /// nominal perigee)`. The frontend compares the plan against its live knobs
    /// so it can grey a result the operator has since tuned away from, rather
    /// than presenting it as current.
    tow_probe: Option<(TractorPlan, f64, f64)>,
    /// Which cell the in-flight mass solve is for, `(launch, arrival)`.
    ///
    /// **No vehicle index, and that is the point.** The requirement is a property of
    /// the *window* — its arrival geometry and its lead — not of whatever rocket is
    /// selected; the frontend divides it by the launcher's payload to get the ratio.
    /// Keying it by vehicle would invite re-solving 30 s of propagation on a `[L]`
    /// press that cannot change the answer.
    pending_mass: (i64, i64),
    /// The last mass requirement and the cell it belongs to. Same staleness
    /// discipline as `verdict`: shown only against the cell it was solved for.
    mass_requirement: Option<(i64, i64, MassSolveOutcome)>,
    /// The in-flight one-period required-Δv anchor solve — a **seventh** channel.
    ///
    /// Unlike the other six this one is not fired repeatedly: it is asked once per
    /// *orbit*, because that is what it describes. It gets its own channel anyway,
    /// on the same principle — a rebuilt threat wants its requirement measured, and
    /// that must not block or be blocked by the grid the operator rebuilds next.
    ///
    /// No pending-identity field beside it, and that is deliberate rather than an
    /// omission: `begin_required_dv_anchor` cannot start while a build is in
    /// flight and a build cannot start while this is, so the orbit it was solved
    /// for is necessarily still the installed one when it lands. The pairing the
    /// other workers need is enforced here by exclusion instead.
    anchor_build: Option<mpsc::Receiver<Result<f64, String>>>,
    /// The in-flight Tier-3 sensitivity solve — an **eighth** independent channel,
    /// for the reason every one before it got its own: the b-plane's uncertainty
    /// layer is asked for from a different screen than the grid, the verify and the
    /// tow probe, and none of them should be able to block or cancel another.
    tier3_build: Option<mpsc::Receiver<Result<Tier3View, String>>>,
    /// The solved sensitivity — 13 propagations' worth of `∂(b-plane)/∂(state)`,
    /// plus the rotation onto the view's axes.
    ///
    /// Held rather than re-solved because holding it is the point: mapping a
    /// covariance through it is free, so the σ knob below costs nothing per press.
    /// Dropped whenever a new scenario is installed, on exactly the reasoning
    /// `porkchop` is dropped there — a Jacobian is about *one* rock's trajectory,
    /// and `[N]` can put a different rock on a different orbit between one frame
    /// and the next.
    tier3: Option<Tier3View>,
    /// How far the σ knob is turned, in **decades** — the covariance is the
    /// probe's, with both blocks multiplied by `10^this`.
    ///
    /// Stored as the exponent rather than the multiplier so gdext's derived `init`
    /// gives the right default for free: `0.0` is `10^0 = 1`, the shipping
    /// covariance exactly. A multiplier field would default to `0.0`, i.e. an
    /// orbit known perfectly, which is both wrong and the flattering direction.
    tier3_sigma_log10: f64,
    /// Whether the **most recent** build attempt failed.
    ///
    /// Distinct from `!is_ready()`, and the distinction only started existing when
    /// rebuilds did. `is_ready()` asks "is a scenario installed", which after a
    /// failed *rebuild* is still **true** — the previous threat is untouched and
    /// still correct. A frontend using `!is_ready()` as its failure test therefore
    /// sees a rebuild that blew up as a success and announces a new threat
    /// solution that was never built.
    build_failed: bool,
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

    /// Read the DE kernels **on a worker thread** and return immediately. The
    /// frontend's real entry point; [`load_from`](Self::load_from) remains for
    /// tests and shell runs, which have nothing to keep responsive.
    ///
    /// Reads the DE pair and nothing else. Arming the small-body kernel stays a
    /// main-thread call *after* this lands ([`set_small_body_kernel`] is a path
    /// stat, not a read), so its warn-and-continue branch — a missing small-body
    /// kernel is a valid machine, not a failed mission — keeps working exactly as
    /// written instead of being smuggled back through this channel.
    ///
    /// Returns `false` + [`last_error`](Self::last_error) if a load is already in
    /// flight or the kernels are already loaded. Drive it with
    /// [`poll_load`](Self::poll_load).
    ///
    /// # Why this is threaded when `load_from` is 27 ms warm
    /// Because warm is not the case that hurts. This is the **first** thing the
    /// frontend does, before a single frame is drawn, so on a cold file cache the
    /// window does not exist yet — the operator gets a black rectangle for the
    /// duration rather than a display that says what it is waiting for. The read
    /// was measured at 2.4–6.4 s off this machine's disk. Nothing here makes the
    /// disk faster; it moves the wait somewhere the display can narrate it.
    ///
    /// The worker returns the whole [`MissionCore`] by value rather than filling
    /// this one in place: a half-loaded core reachable from the main thread is
    /// exactly the state every other accessor here is written to assume cannot
    /// exist, and `core` staying `None` until the worker lands preserves that.
    #[func]
    fn begin_load(&mut self, bsp_path: GString, pca_path: GString) -> bool {
        if self.field_load.is_some() {
            self.error = "a kernel load is already in flight".into();
            return false;
        }
        if self.core.is_some() {
            self.error = "the kernels are already loaded".into();
            return false;
        }
        let (bsp, pca) = (bsp_path.to_string(), pca_path.to_string());
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = MissionCore::load_from(&bsp, &pca).map_err(|e| e.to_string());
            // A closed channel means the game quit mid-read: drop it, never panic on
            // a detached thread. Same contract as every other worker here.
            let _ = tx.send(result);
        });
        self.error = GString::new();
        self.field_load = Some(rx);
        true
    }

    /// Whether the kernel read is still running.
    #[func]
    fn is_loading(&self) -> bool {
        self.field_load.is_some()
    }

    /// Pump the kernel read: adopt the core if it has landed. Returns `true` while
    /// it is **still reading**, `false` once it has finished or was never started —
    /// the same shape as [`poll_catalog`](Self::poll_catalog), so the frontend
    /// drives it with the polling loop it already has.
    ///
    /// `false` means *finished*, not *succeeded*: ask
    /// [`is_loaded`](Self::is_loaded) for that, and
    /// [`last_error`](Self::last_error) for why not.
    ///
    /// On failure the core stays `None` and the reason lands in
    /// [`last_error`](Self::last_error), which is the contract
    /// [`finish_load`](Self::finish_load) already pins for the synchronous path: a
    /// failed load never leaves a half-usable core behind.
    ///
    /// Non-blocking, so it is safe to call every frame.
    #[func]
    fn poll_load(&mut self) -> bool {
        let Some(rx) = self.field_load.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(core)) => {
                self.field_load = None;
                self.core = Some(core);
                self.error = GString::new();
                false
            }
            Ok(Err(message)) => {
                self.field_load = None;
                self.core = None;
                self.error = GString::from(&message);
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.field_load = None;
                self.core = None;
                self.error = "the kernel read thread died without reporting".into();
                false
            }
        }
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
        self.core.as_ref().is_some_and(|c| c.small_bodies_mounted())
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
        self.spawn_build(ImpactorConfig::default())
    }

    /// Rebuild the campaign with the threat on a **different heliocentric orbit**.
    ///
    /// Same worker, same ~10 s, same [`poll_build`](Self::poll_build) — the only
    /// difference is the config. `azimuth_deg`/`elevation_deg` give the approach
    /// direction; see [`ThreatOrbitKnobs`] for why those three knobs and not the
    /// other two.
    ///
    /// # It refuses while *any* other worker is in flight, and that is not caution
    /// The porkchop grid, the Tier-2 preview, the cell verify, the mass solve and
    /// the tow probe each hold an `Arc` clone of the **current** scenario. Start a
    /// rebuild underneath one and it keeps computing — correctly — about a threat
    /// that no longer exists, then lands *after* `poll_build` has cleared the
    /// state it belongs to and installs itself as current. Every one of those is a
    /// number attributed to the wrong orbit.
    ///
    /// Refusing by name is the cheap fix and the honest one: the operator is told
    /// what to wait for. A generation counter threaded through six result types
    /// would be the expensive fix, and it would still have to say the same thing.
    ///
    /// Returns `false` + [`last_error`](Self::last_error) if something is running,
    /// the kernels are not loaded, or the geometry is one
    /// [`ImpactorConfig::preview`] already knows the builder would reject — the
    /// last of which is the whole reason the preview exists, since the builder
    /// charges 10 s to reach the same verdict.
    #[func]
    fn begin_rebuild_scenario(
        &mut self,
        v_rel_kms: f64,
        azimuth_deg: f64,
        elevation_deg: f64,
        b_offset_km: f64,
    ) -> bool {
        if let Some(busy) = self.busy_worker() {
            self.error = format!(
                "cannot rebuild the threat while {busy} is running — it is solving \
                 against the orbit that is about to be replaced"
            )
            .as_str()
            .into();
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_rebuild_scenario()".into();
            return false;
        };
        let cfg = ThreatOrbitKnobs {
            v_rel_kms,
            azimuth_deg,
            elevation_deg,
            b_offset_km,
        }
        .to_config();

        // The cheap wall in front of the expensive one. `preview` reaches the same
        // two rejections in microseconds that `build_with` reaches in ~10 s.
        match cfg.preview(&core.ephemeris_arc()) {
            Err(e) => {
                self.error = e.to_string().as_str().into();
                return false;
            }
            Ok(p) if !p.is_hit => {
                self.error = format!(
                    "this geometry misses Earth: the incoming asymptote passes \
                     {:.0} km from the centre, outside the {:.0} km capture disc — \
                     there is no impact to deflect",
                    p.impact_parameter / 1000.0,
                    p.capture_radius / 1000.0,
                )
                .as_str()
                .into();
                return false;
            }
            Ok(_) => {}
        }
        self.spawn_build(cfg)
    }

    /// The shared body of [`begin_build_scenario`](Self::begin_build_scenario) and
    /// [`begin_rebuild_scenario`](Self::begin_rebuild_scenario) — one worker, one
    /// small-body mount, one catalog seed, parameterised only by the config.
    ///
    /// Extracted rather than copied: the boot path and the rebuild path must
    /// produce scenarios that differ *only* in their orbit. A second worker written
    /// beside this one would be free to drift in what it mounts or what it seeds,
    /// and the difference would show up as a rebuilt threat with no comet.
    fn spawn_build(&mut self, cfg: ImpactorConfig) -> bool {
        if self.build.is_some() {
            self.error = "a scenario build is already in flight".into();
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before building a scenario".into();
            return false;
        };
        // A new attempt clears the previous verdict, so a stale failure cannot be
        // read as this build's.
        self.build_failed = false;
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
            let result = BuiltScenario::build(Arc::clone(&eph), &cfg, mounted)
                // The Tier-2 shift preview is DELIBERATELY not measured here: it is ~64 s
                // of propagation that would sit *before* `install`, delaying the threat
                // solution and the planner — the core gameplay — by that much. It is
                // instead computed on demand when the operator opens the force-model menu
                // (`begin_tier2_preview`), off the same scenario, so the threat lands as
                // fast as it did before the menu existed.
                .map_err(|e| e.to_string())
                .map(|built| {
                    // The real asteroids join the catalog here — and they cost no
                    // integration at all. A `.neo` table already holds JPL's
                    // trajectory, so this is a file read (milliseconds). It rides
                    // the worker because this is where the catalog is assembled,
                    // not because it is expensive.
                    //
                    // Absent tables are the ordinary state of a fresh clone and
                    // produce an empty vector, exactly as an unmounted small-body
                    // kernel produces an empty asteroid list.
                    //
                    // The display comet used to fly here too, and that was ~4 s the
                    // threat solution waited on for a piece of scenery. It now flies
                    // on its own worker, started once this scenario is installed —
                    // see [`spawn_comet`](Self::spawn_comet).
                    (built, load_neo_bodies())
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
    /// at which point [`last_build_failed`](Self::last_build_failed) says whether
    /// it succeeded and [`last_error`](Self::last_error) says why if it did not.
    ///
    /// **Ask `last_build_failed`, not `!is_ready`.** They agree on a first build
    /// and disagree on a rebuild, where a failure leaves the previous scenario
    /// installed and `is_ready()` perfectly true.
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
                let mut installed = false;
                match self.core.as_mut() {
                    Some(core) => {
                        core.install(built, bodies);
                        installed = true;
                        // A porkchop belongs to the scenario it was solved against —
                        // its axes come from that campaign's epochs and its cells
                        // from that nominal trajectory. Installing a new scenario
                        // makes the old grid a picture of a threat that is no longer
                        // there, so it is dropped rather than left to be read.
                        self.porkchop = None;
                        self.verdict = None;
                        self.mass_requirement = None;
                        // …and so does a tow probe. It is a perigee reached by
                        // towing *this* rock through *this* field over *this*
                        // trajectory; against a rebuilt threat it is a measurement
                        // of a mission that no longer exists. It survived here
                        // until the threat orbit became dialable, when a stale
                        // probe stopped being unreachable and became one keypress
                        // away.
                        self.tow_probe = None;
                        // …and the Tier-3 sensitivity, for the sharpest version of
                        // the same reason. It is a Jacobian about one seed state:
                        // against a rebuilt threat it would keep drawing the old
                        // rock's ellipse on the new rock's b-plane, at axis lengths
                        // and an angle that are individually plausible and jointly
                        // about nothing. The σ knob is deliberately *not* reset —
                        // "how well is the orbit known" is an operator's question
                        // about the layer, not a property of any one threat.
                        self.tier3 = None;
                        self.error = GString::new();
                    }
                    // The kernels were dropped (a failed re-load) while the build
                    // ran, so there is nothing to install it into. Say so rather
                    // than discard it silently and read as "still not ready".
                    None => {
                        self.build_failed = true;
                        self.error =
                            "the scenario finished building but the kernels are no longer loaded"
                                .into()
                    }
                }
                // The comet flies from HERE, against the scenario that was just
                // installed — after the threat is online, not before it. Everything
                // this call needs (the scenario, the field) is an `Arc` on the core,
                // so it costs a refcount bump on this thread and ~4 s on another.
                if installed {
                    self.spawn_comet();
                }
                false
            }
            Ok(Err(message)) => {
                self.build = None;
                self.build_failed = true;
                self.error = message.as_str().into();
                false
            }
            // The worker panicked and took the sender with it. A build that dies
            // without a word must not leave the frontend polling forever.
            Err(mpsc::TryRecvError::Disconnected) => {
                self.build = None;
                self.build_failed = true;
                self.error = "the scenario build thread died without reporting".into();
                false
            }
        }
    }

    /// Fly the display comet on its own worker, in the field of the scenario that
    /// was just installed.
    ///
    /// # Why it is not on the build worker any more
    /// The build phases were measured (`build_phase_timings`, release): the
    /// small-body mount is under a second, `BuiltScenario::build` is 11.8 s of
    /// which 10.7 s is the forward nominal flight, and the comet is 4.1 s. The
    /// comet was the only one of those with anywhere to go — the mount feeds the
    /// build (`BuiltScenario::build` consumes the mounted almanac and the scenario
    /// keeps it, which is what the force-model menu recomposes from), and the
    /// forward flight *is* the build's own hit check. So the comet moved off the
    /// path instead, where it costs the threat solution nothing at all.
    ///
    /// # A comet that does not fly is not a failed mission
    /// It used to be. On the build worker the flight was fallible with `?`, so a
    /// comet that would not fly took the whole threat solution down with it — over
    /// a piece of scenery, and against what `sim.gd` has always documented
    /// (`comet_online` is "set from what the catalog actually holds… the comet is a
    /// separate body that can fail to fly on its own"). Here a failure warns and
    /// leaves the catalog without a comet, the same stance the small-body mount
    /// already took: a missing catalog beats a missing threat.
    ///
    /// Staleness is handled by refusal, not by a generation counter: this worker
    /// holds an `Arc` of the installed scenario, and
    /// [`busy_worker`](Self::busy_worker) names it, so a threat rebuild cannot
    /// start underneath it and inherit a comet flown in the previous field.
    fn spawn_comet(&mut self) {
        let Some(core) = self.core.as_ref() else {
            return;
        };
        // No scenario means nothing to fly the comet *in*. Unreachable from
        // `poll_build`, which only calls this after a successful `install`, but this
        // returns rather than unwraps: a panic here would cross the FFI.
        let Some(scenario) = core.scenario_arc() else {
            return;
        };
        let eph = core.ephemeris_arc();
        let epoch0 = scenario.epoch0();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = seed_orrery_body(
                &eph,
                &scenario,
                display_comet::NAME,
                display_comet::KIND,
                display_comet::elements(),
                epoch0,
                display_comet::CADENCE_SECONDS,
                display_comet::N_SNAPSHOTS,
            )
            .map_err(|e| e.to_string());
            // A closed channel means the game quit mid-flight, exactly as on the
            // build worker: drop the result, never panic on a detached thread.
            let _ = tx.send(result);
        });
        self.comet_build = Some(rx);
    }

    /// Whether the display comet is still flying on its worker.
    #[func]
    fn is_catalog_building(&self) -> bool {
        self.comet_build.is_some()
    }

    /// Pump the comet's flight: add it to the catalog if it has landed. Returns
    /// `true` while it is **still flying**, `false` once it is finished or was
    /// never started.
    ///
    /// The frontend re-reads the catalog when this goes false, because the comet
    /// was not in it when the threat came online. Nothing here can fail the
    /// mission — see [`spawn_comet`](Self::spawn_comet) — so there is no
    /// `last_build_failed` equivalent to ask afterwards; the catalog either has a
    /// comet in it or does not, which is the question the frontend was already
    /// asking.
    ///
    /// Non-blocking, so it is safe to call every frame.
    #[func]
    fn poll_catalog(&mut self) -> bool {
        let Some(rx) = self.comet_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(body)) => {
                self.comet_build = None;
                match self.core.as_mut() {
                    Some(core) => {
                        core.adopt_orrery_body(body);
                    }
                    // The kernels were dropped while it flew, so there is no catalog
                    // to land in. Scenery, so it warns rather than failing anything.
                    None => godot_warn!(
                        "the display comet finished flying but the kernels are no longer loaded"
                    ),
                }
                false
            }
            Ok(Err(message)) => {
                self.comet_build = None;
                godot_warn!("display comet not flown, the catalog will have no comet: {message}");
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.comet_build = None;
                godot_warn!("the display comet's flight thread died without reporting");
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
                // axes may differ, so the same (i, j) is a different window. The mass
                // requirement is keyed the same way and goes for the same reason.
                self.verdict = None;
                self.mass_requirement = None;
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

    // --- The Tier-3 uncertainty ellipse (HANDOFF §7) ------------------------

    /// Solve the b-plane sensitivity on a worker — the expensive half of the
    /// uncertainty layer, paid once.
    ///
    /// **13 propagations, ~17 s.** Everything after it is free: the ellipse for any
    /// covariance is a 2×2 matrix product, which is why the σ knob can be on the
    /// arrow keys. Needs a built scenario, not merely kernels.
    ///
    /// Returns `false` — with a reason in [`last_error`](Self::last_error) — if a
    /// solve is already running or there is nothing to solve against.
    #[func]
    fn begin_tier3(&mut self) -> bool {
        if self.tier3_build.is_some() {
            return false; // already solving — not an error
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_tier3()".into();
            return false;
        };
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before solving the Tier-3 ellipse".into();
            return false;
        };
        // The axes the picture will use, captured now and carried to the worker —
        // see `MissionCore::display_basis` for why they are not re-derived there.
        let Some(basis) = core.display_basis() else {
            self.error = "the encounter has no b-plane frame to draw an ellipse in".into();
            return false;
        };
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = Tier3View::build(&scenario, basis).map_err(|e| e.to_string());
            let _ = tx.send(result);
        });
        self.tier3_build = Some(rx);
        self.error = GString::new();
        true
    }

    /// Whether the Tier-3 sensitivity solve is in flight.
    #[func]
    fn is_solving_tier3(&self) -> bool {
        self.tier3_build.is_some()
    }

    /// Pump the Tier-3 worker. `true` while **still running**, `false` once
    /// finished (or none in flight) — then [`has_tier3`](Self::has_tier3) says
    /// whether it succeeded. Non-blocking; safe every frame.
    #[func]
    fn poll_tier3(&mut self) -> bool {
        let Some(rx) = self.tier3_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(view)) => {
                self.tier3_build = None;
                self.tier3 = Some(view);
                self.error = GString::new();
                false
            }
            Ok(Err(message)) => {
                self.tier3_build = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.tier3_build = None;
                self.error = "the Tier-3 solve thread died without reporting".into();
                false
            }
        }
    }

    /// Whether a solved sensitivity is available to map covariances through.
    #[func]
    fn has_tier3(&self) -> bool {
        self.tier3.is_some()
    }

    /// Turn the σ knob to `decades` — the synthetic covariance with both blocks
    /// multiplied by `10^decades`. `0.0` is the shipping covariance.
    ///
    /// Free: no propagation, and the next
    /// [`tier3_ellipse`](Self::tier3_ellipse) read reflects it. That is the whole
    /// argument for holding the Jacobian, and the comparison it buys — the same
    /// rock, the same trajectory, an impact probability that moves orders of
    /// magnitude purely on how long anyone has been watching — is the Tier-3
    /// lesson the deterministic view cannot show.
    #[func]
    fn tier3_set_sigma_log10(&mut self, decades: f64) {
        if decades.is_finite() {
            self.tier3_sigma_log10 = decades;
        }
    }

    /// Where the σ knob is, in decades.
    #[func]
    fn tier3_sigma_log10(&self) -> f64 {
        self.tier3_sigma_log10
    }

    /// The 1σ b-plane ellipse at the current σ knob, in the **view's own** `(ξ, ζ)`
    /// kilometres — or an empty dictionary before the sensitivity is solved.
    ///
    /// `mean_xi_km` / `mean_zeta_km` centre it. **That centre is the fixed-epoch
    /// reduction, which is not the reduction the drawn nominal cross uses** — the
    /// two differ by `mean_gap_km`, reported here so a caller can decide rather
    /// than discover. `frame_residual` and `out_of_plane_km` are the coplanarity
    /// checks on the rotation (see `Tier3View`).
    ///
    /// `p_impact` is over the disc of `capture_km`, and the pair has to be read
    /// together: the campaign's designed hit is thousands of ellipse-widths from
    /// Earth's *centre* and still `p_impact = 1`, because all of that sits inside a
    /// disc 11 000 km wide. `sigma_distance` is deliberately **not** exposed —
    /// on its own it reads as "how many σ from a hit" and inverts the answer.
    #[func]
    fn tier3_ellipse(&self) -> VarDictionary {
        let mut d = VarDictionary::new();
        let (Some(view), Some(core)) = (self.tier3.as_ref(), self.core.as_ref()) else {
            return d;
        };
        let Some(e) = view.map(10.0_f64.powf(self.tier3_sigma_log10)) else {
            return d;
        };
        d.set("mean_xi_km", e.mean_km.0);
        d.set("mean_zeta_km", e.mean_km.1);
        d.set("major_km", e.major_km);
        d.set("minor_km", e.minor_km);
        d.set("angle_rad", e.angle_rad);
        d.set("capture_km", e.capture_km);
        d.set("p_impact", e.impact_probability);
        d.set("sigma_scale", e.sigma_scale);
        d.set("frame_residual", view.frame_residual());
        d.set("out_of_plane_km", view.out_of_plane_km());
        // How far the ellipse's own centre sits from the cross the view draws.
        // Both are honest reductions of one hyperbola; they are not one number, and
        // a picture that silently pairs them is a picture pairing two instruments.
        if let Some(b) = core.nominal_b_point_km() {
            d.set(
                "mean_gap_km",
                ((b.x - e.mean_km.0).powi(2) + (b.y - e.mean_km.1).powi(2)).sqrt(),
            );
        }
        d
    }

    /// Rows in the grid (launch epochs); `0` if none is built.
    #[func]
    fn porkchop_launch_count(&self) -> i64 {
        self.porkchop
            .as_ref()
            .map_or(0, |p| p.launch_count() as i64)
    }

    /// Columns in the grid (arrival epochs); `0` if none is built.
    #[func]
    fn porkchop_arrival_count(&self) -> i64 {
        self.porkchop
            .as_ref()
            .map_or(0, |p| p.arrival_count() as i64)
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

    // --- The on-demand required-impactor-mass solve -------------------------

    /// Solve, on a worker thread, for the impactor mass this window would need to
    /// reach the campaign's safe-perigee target — the `[M]` half of the readout, and
    /// the question `[E]` cannot answer.
    ///
    /// `[E]` asks "does *this launcher* through *this window* work", and when the
    /// answer is no it says so and stops. This asks the follow-up an operator
    /// actually needs — *how far off is it* — and answers in a unit that composes
    /// with the launcher's payload: a ratio. That ratio is the campaign's honest
    /// headline (a real launcher delivers a small fraction of what the curve wants),
    /// and until now the frontend had no way to state it.
    ///
    /// **Vehicle-independent**, so it takes no vehicle index: see
    /// [`pending_mass`](Self::pending_mass). Many full-field propagations, so it is
    /// far slower than a verify — the frontend must show that it is running.
    ///
    /// Returns `false` — with a reason in [`last_error`](Self::last_error) — when a
    /// solve is already in flight, there is no grid or scenario, the indices are out
    /// of range, or the cell carries no transfer.
    #[func]
    fn begin_required_mass(&mut self, i: i64, j: i64) -> bool {
        if self.mass_build.is_some() {
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_required_mass()".into();
            return false;
        };
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before solving for a required mass".into();
            return false;
        };
        let Some(p) = self.porkchop.as_ref() else {
            self.error = "no porkchop grid to solve against".into();
            return false;
        };
        if i < 0 || j < 0 {
            self.error = "cell indices must be non-negative".into();
            return false;
        }
        // The *arrival* epoch and the metrics are all this needs — no launcher, no
        // payload. A cell with no transfer has no geometry to solve against, which is
        // a different thing from a window no rocket can reach.
        let (Some(metrics), Some(arrival_tdb)) = (
            p.metrics_at(i as usize, j as usize),
            p.arrival_tdb().get(j as usize).copied(),
        ) else {
            self.error = "that cell carries no transfer to solve".into();
            return false;
        };
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result =
                required_cell_mass(&scenario, arrival_tdb, &metrics).map_err(|e| e.to_string());
            let _ = tx.send(result);
        });
        self.mass_build = Some(rx);
        // Drop any previous requirement: one shown beside a running solve would read
        // as this solve's answer.
        self.pending_mass = (i, j);
        self.mass_requirement = None;
        self.error = GString::new();
        true
    }

    /// Whether the required-mass solve is in flight.
    #[func]
    fn is_solving_required_mass(&self) -> bool {
        self.mass_build.is_some()
    }

    /// Pump the required-mass worker. `true` while **still running**, `false` once
    /// finished (or none in flight) — then [`required_mass`](Self::required_mass)
    /// holds the result. Non-blocking; safe every frame.
    #[func]
    fn poll_required_mass(&mut self) -> bool {
        let Some(rx) = self.mass_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(outcome)) => {
                self.mass_build = None;
                let (i, j) = self.pending_mass;
                self.mass_requirement = Some((i, j, outcome));
                self.error = GString::new();
                false
            }
            Ok(Err(message)) => {
                self.mass_build = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.mass_build = None;
                self.error = "the required-mass thread died without reporting".into();
                false
            }
        }
    }

    /// The last required-impactor-mass result, or an **empty dictionary** if none has
    /// been solved for the current grid.
    ///
    /// Keys always present: `launch_index`, `arrival_index` (which window this
    /// describes — no vehicle, because the answer does not depend on one),
    /// `target_perigee_m` (the bar it was solved against, which a readout **must**
    /// name — see [`SAFE_PERIGEE_TARGET_M`]), and `outcome`, one of:
    ///
    /// - `"feasible"` — plus `impactor_mass_kg`, the mass that just clears the
    ///   target through this window.
    /// - `"infeasible_at_cap"` — plus `mass_cap_kg` and `perigee_reached_m`. **This
    ///   is an expected answer, not a failure**: it is the honest state of a window
    ///   too weakly coupled or too late to be saved by any mass worth launching, and
    ///   the perigee it did reach says how close it got. A display that renders it as
    ///   an error repeats this project's clean-miss-shares-a-sentinel bug in a new
    ///   place.
    #[func]
    fn required_mass(&self) -> VarDictionary {
        let mut d = VarDictionary::new();
        let Some((i, j, outcome)) = self.mass_requirement else {
            return d;
        };
        d.set("launch_index", i);
        d.set("arrival_index", j);
        d.set("target_perigee_m", SAFE_PERIGEE_TARGET_M);
        match outcome {
            MassSolveOutcome::Feasible { impactor_mass_kg } => {
                d.set("outcome", "feasible");
                d.set("impactor_mass_kg", impactor_mass_kg);
            }
            MassSolveOutcome::InfeasibleAtCap {
                mass_cap_kg,
                perigee_reached_m,
            } => {
                d.set("outcome", "infeasible_at_cap");
                d.set("mass_cap_kg", mass_cap_kg);
                d.set("perigee_reached_m", perigee_reached_m);
            }
        }
        d
    }

    /// The heaviest mass any launcher on the table can deliver, kg — the unit an
    /// `infeasible_at_cap` reading is quoted in ("more than N of the best rocket
    /// there is"), so the display need not restate the AMAT/LSP tables to say it.
    #[func]
    fn heaviest_deliverable_kg(&self) -> f64 {
        heaviest_deliverable_kg()
    }

    // --- The threat orbit ------------------------------------------------------

    /// Which named worker is running, for a refusal message that says what to wait
    /// for. `None` when the core is idle.
    ///
    /// Order is worst-first: if several are somehow in flight, the message names
    /// the longest one rather than whichever happens to be checked first.
    fn busy_worker(&self) -> Option<&'static str> {
        if self.build.is_some() {
            Some("a scenario build")
        } else if self.comet_build.is_some() {
            Some("the display comet's flight")
        } else if self.anchor_build.is_some() {
            Some("the required-Δv anchor solve")
        } else if self.tier2_build.is_some() {
            Some("the Tier-2 force-model preview")
        } else if self.porkchop_build.is_some() {
            Some("the launch-window map build")
        } else if self.mass_build.is_some() {
            Some("the required-mass solve")
        } else if self.verify_build.is_some() {
            Some("a launch-window cell verify")
        } else if self.tow_build.is_some() {
            Some("a gravity-tractor probe")
        } else {
            None
        }
    }

    /// The shipping threat orbit as knobs, plus the bounds a control must respect.
    ///
    /// Same contract as [`tractor_defaults`](Self::tractor_defaults): GDScript
    /// restates no physics constant. The shipping values are *derived from*
    /// `ImpactorConfig::default()` rather than copied, so the panel cannot open on
    /// numbers that merely used to match the config.
    ///
    /// `max_b_offset_km` is Earth's equatorial radius, and it is a real bound, not
    /// a tidy one: the impact offset is laid perpendicular to the relative
    /// velocity, so it *is* the geocentric perigee, and past `R⊕` the designed
    /// impact stops being an impact. There is deliberately **no**
    /// `min_b_offset_km`, because that wall moves with `v_rel_kms` (Earth escape at
    /// the offset distance) and a fixed number would be wrong at every other speed
    /// — [`threat_orbit_preview`](Self::threat_orbit_preview) reports it live
    /// instead.
    #[func]
    fn threat_orbit_defaults(&self) -> VarDictionary {
        let k = ThreatOrbitKnobs::shipping();
        let mut d = VarDictionary::new();
        d.set("v_rel_kms", k.v_rel_kms);
        d.set("azimuth_deg", k.azimuth_deg);
        d.set("elevation_deg", k.elevation_deg);
        d.set("b_offset_km", k.b_offset_km);
        d.set(
            "max_b_offset_km",
            asteroid_core::geometry::EARTH_EQUATORIAL_RADIUS_M / 1000.0,
        );
        d
    }

    /// Which threat orbit is **installed**, as knobs — empty before the first
    /// build.
    ///
    /// Distinct from [`threat_orbit_defaults`](Self::threat_orbit_defaults) the
    /// moment a rebuild lands, and that difference is the point: a panel showing
    /// the knobs it last sent is showing an intention, while this is the orbit the
    /// numbers beside it were actually computed on. `is_shipping` is what decides
    /// whether the pinned required-Δv anchor still applies.
    #[func]
    fn threat_orbit(&self) -> VarDictionary {
        let mut d = VarDictionary::new();
        let Some(k) = self.core.as_ref().and_then(MissionCore::threat_orbit_knobs) else {
            return d;
        };
        d.set("v_rel_kms", k.v_rel_kms);
        d.set("azimuth_deg", k.azimuth_deg);
        d.set("elevation_deg", k.elevation_deg);
        d.set("b_offset_km", k.b_offset_km);
        d.set("is_shipping", k.is_shipping());
        d
    }

    /// What a set of knobs would produce, in **closed form** — free, and safe to
    /// call every frame while a knob is turning.
    ///
    /// This is the whole reason the threat orbit is dialable at all. A rebuild is
    /// ~10 s and can fail two different ways; without a preview the panel would be
    /// a set of knobs whose only feedback is a ten-second wait followed by an
    /// error. With it, the operator watches the orbit change as they turn.
    ///
    /// Keys always present: `ok`. When `ok`: `v_inf_m_s`, `impact_parameter_m`,
    /// `capture_radius_m`, `is_hit`, `semi_major_axis_m`, `eccentricity`,
    /// `inclination_deg`, `period_seconds`. When not: `error`.
    ///
    /// # `is_hit` is false, not an error
    /// A geometry that has stopped being an impact still has a perfectly good
    /// `v_inf` and a perfectly good miss distance, and those two numbers are
    /// exactly what explains *why* it stopped. Collapsing that to a refusal would
    /// throw away the explanation and leave the operator guessing which knob to
    /// turn back.
    ///
    /// # `period_seconds` labels; it does not score
    /// Osculating at the impact epoch, against a build that reports vis-viva at the
    /// seed 12 years earlier — measured 0.23 % apart at worst. Fine on a readout,
    /// not fine divided into a lead to form a margin. The tractor bench keeps
    /// taking its period from the built scenario.
    #[func]
    fn threat_orbit_preview(
        &self,
        v_rel_kms: f64,
        azimuth_deg: f64,
        elevation_deg: f64,
        b_offset_km: f64,
    ) -> VarDictionary {
        let mut d = VarDictionary::new();
        let Some(core) = self.core.as_ref() else {
            d.set("ok", false);
            d.set("error", "kernels are not loaded");
            return d;
        };
        let cfg = ThreatOrbitKnobs {
            v_rel_kms,
            azimuth_deg,
            elevation_deg,
            b_offset_km,
        }
        .to_config();
        match cfg.preview(&core.ephemeris_arc()) {
            Ok(p) => {
                d.set("ok", true);
                d.set("v_inf_m_s", p.v_inf);
                d.set("impact_parameter_m", p.impact_parameter);
                d.set("capture_radius_m", p.capture_radius);
                d.set("is_hit", p.is_hit);
                d.set("semi_major_axis_m", p.semi_major_axis_m);
                d.set("eccentricity", p.eccentricity);
                d.set("inclination_deg", p.inclination_rad.to_degrees());
                d.set("period_seconds", p.period_seconds);
            }
            Err(e) => {
                d.set("ok", false);
                d.set("error", e.to_string().as_str());
            }
        }
        d
    }

    // --- The required-Δv anchor ------------------------------------------------

    /// The one-period required-Δv anchor for the **installed** orbit, m/s, or `0`
    /// if this orbit's requirement has not been measured.
    ///
    /// Zero is a safe sentinel here only because a real anchor cannot be zero — a
    /// threat that needs no deflection is not a threat — and
    /// `adopt_required_dv_anchor` refuses to store one. Callers should still prefer
    /// [`has_required_dv_anchor`](Self::has_required_dv_anchor), which says the
    /// same thing without asking anyone to know that.
    #[func]
    fn required_dv_anchor(&self) -> f64 {
        self.core
            .as_ref()
            .and_then(MissionCore::required_dv_anchor)
            .unwrap_or(0.0)
    }

    /// Whether the installed orbit's one-period requirement has been measured.
    ///
    /// `true` immediately on the shipping orbit — the constant *is* that
    /// measurement, recorded — and `false` after a rebuild until
    /// [`begin_required_dv_anchor`](Self::begin_required_dv_anchor) lands.
    #[func]
    fn has_required_dv_anchor(&self) -> bool {
        self.core
            .as_ref()
            .and_then(MissionCore::required_dv_anchor)
            .is_some()
    }

    /// Solve this orbit's one-period required Δv on a worker thread.
    ///
    /// **The expensive half of a rebuild, and it is separate on purpose.**
    /// Measured **28.8 s on the shipping orbit** (0.79 yr period) and **41–74 s on
    /// a 1.44 yr one**, against ~10 s for the build itself. The cost **scales with
    /// the period**, because a one-period lead on a longer orbit is a
    /// proportionally longer propagation — so the long-period orbits an operator
    /// is most likely to go looking for are the slow ones to score, and the knobs
    /// reach past 3 yr.
    ///
    /// The scenario build gives back a threat that can be drawn, planned against
    /// and probed; only the tractor bench's *margin* needs this. Folding it into
    /// the build would make every rebuild wait three to six times as long for a
    /// number most of the frontend does not use — the same argument that keeps the
    /// Tier-2 preview off the build path.
    ///
    /// That is also why the frontend must show this running rather than merely go
    /// quiet, and why its copy says "about a minute" rather than a range: the
    /// first draft promised "30–60 s", the next run took 63, and the one after
    /// that 74. A number measured on the one orbit nobody rebuilds *to* is
    /// accurate and still a misleading promise.
    ///
    /// Refuses while any other worker is running, for the reason
    /// [`begin_rebuild_scenario`](Self::begin_rebuild_scenario) documents.
    #[func]
    fn begin_required_dv_anchor(&mut self) -> bool {
        if let Some(busy) = self.busy_worker() {
            self.error = format!("cannot solve the anchor while {busy} is running")
                .as_str()
                .into();
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "build the scenario before solving for its required Δv".into();
            return false;
        };
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before solving for its required Δv".into();
            return false;
        };
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(solve_required_dv_anchor(&scenario).map_err(|e| e.to_string()));
        });
        self.anchor_build = Some(rx);
        self.error = GString::new();
        true
    }

    /// Whether the anchor solve is in flight.
    #[func]
    fn is_solving_required_dv_anchor(&self) -> bool {
        self.anchor_build.is_some()
    }

    /// Pump the anchor worker. `true` while **still running**, `false` once
    /// finished (or none in flight) — then
    /// [`has_required_dv_anchor`](Self::has_required_dv_anchor) says whether it
    /// succeeded. Non-blocking; safe every frame.
    #[func]
    fn poll_required_dv_anchor(&mut self) -> bool {
        let Some(rx) = self.anchor_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok(dv)) => {
                self.anchor_build = None;
                match self.core.as_mut() {
                    Some(core) => {
                        if core.adopt_required_dv_anchor(dv) {
                            self.error = GString::new();
                        } else {
                            // A solve that came back non-positive is not an anchor,
                            // and storing it would divide into a margin of zero — a
                            // physics claim, made by arithmetic that failed.
                            self.error = format!(
                                "the anchor solve returned {dv:.4e} m/s, which is not a \
                                 requirement — the margin stays unmeasured"
                            )
                            .as_str()
                            .into();
                        }
                    }
                    None => self.error = "the anchor solved but the kernels are gone".into(),
                }
                false
            }
            Ok(Err(message)) => {
                self.anchor_build = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.anchor_build = None;
                self.error = "the anchor solve thread died without reporting".into();
                false
            }
        }
    }

    // --- The gravity tractor -------------------------------------------------

    /// The shipping tractor configuration and the bounds the panel's knobs move
    /// within: `hover_radii`, `rock_radius_m`, `law_min_periods`.
    ///
    /// Exists so GDScript never restates a physics constant. The frontend already
    /// learned this lesson the expensive way — every drawn body must name a
    /// source — and a panel that hard-coded `1.5` beside a core that had moved on
    /// would be the same bug wearing a different hat.
    #[func]
    fn tractor_defaults(&self) -> VarDictionary {
        let mut d = VarDictionary::new();
        d.set("hover_radii", TRACTOR_HOVER_RADII);
        // The bound a hover control must clamp to — `1/cos(plume)`, ~1.064 radii,
        // NOT 1.0. Supplied because the intuitive bound is the surface and the
        // surface is wrong: between them lies a band that tows and cannot be
        // flown.
        d.set("min_hover_radii", tractor_min_hover_radii());
        d.set("rock_radius_m", THREAT_RADIUS_M);
        d.set("law_min_periods", REQUIRED_DV_LAW_MIN_PERIODS);
        d.set("target_perigee_m", SAFE_PERIGEE_TARGET_M);
        d
    }

    /// Score one tractor configuration with **arithmetic only** — free, exact,
    /// and safe to call every frame while a knob is turning.
    ///
    /// One call returning one dictionary rather than eight getters, so that
    /// adding a knob is one field on each side instead of a new FFI entry point.
    ///
    /// Keys: `flyable` (false when the spacecraft would be inside the body — the
    /// geometric wall that `1/d²` alone never reveals), `tow_accel_m_s2`,
    /// `thrust_n`, `cant_deg`, `rock_mass_kg`, `delivered_dv_m_s`,
    /// `equivalent_dv_m_s`, `lead_periods`, and — **only when the lead is inside
    /// the law's validity range** — `required_dv_m_s` and `margin`.
    ///
    /// # The two Δv keys are not interchangeable
    ///
    /// `delivered_dv_m_s` is `a·T`, what the tow delivers, and a true *upper
    /// bound* on what it is worth. `equivalent_dv_m_s` is the impulsive
    /// equivalent, which is what may be compared against a requirement. A display
    /// that computed its margin from the delivered figure would overstate every
    /// deflection by up to a factor of two — measured at +21 % on the one
    /// configuration with a real-field answer. `margin` is supplied here, already
    /// formed from the right one, so no caller has to make that choice.
    ///
    /// `required_dv_m_s` and `margin` are **absent**, not zero or `NaN`, below
    /// `law_min_periods` — the estimate declines to answer where it would be the
    /// wrong shape rather than merely imprecise.
    #[func]
    fn tractor_readout(
        &self,
        spacecraft_mass_kg: f64,
        hover_radii: f64,
        rock_radius_m: f64,
        lead_seconds: f64,
        duration_seconds: f64,
        retrograde: bool,
    ) -> VarDictionary {
        let plan = TractorPlan {
            spacecraft_mass_kg,
            hover_radii,
            rock_radius_m,
            lead_seconds,
            duration_seconds,
            retrograde,
        };
        let period = self.core.as_ref().map_or(0.0, MissionCore::period_seconds);
        // The anchor is a property of the *installed* orbit, so it comes from the
        // core rather than from a constant. After a rebuild it is `None` until the
        // solve lands, and the two keys below simply do not appear — the same
        // absence a sub-one-period lead produces, and for the same reason.
        let anchor = self.core.as_ref().and_then(MissionCore::required_dv_anchor);
        let r = score_tractor_plan(&plan, period, anchor);
        let mut d = VarDictionary::new();
        d.set("flyable", plan.is_flyable());
        d.set("holds_station", r.holds_station);
        d.set("tow_accel_m_s2", r.tow_acceleration_m_s2.unwrap_or(0.0));
        d.set("thrust_n", r.station_keeping_thrust_n.unwrap_or(0.0));
        d.set("cant_deg", r.cant_angle_rad.map_or(0.0, f64::to_degrees));
        d.set("rock_mass_kg", r.rock_mass_kg);
        d.set("delivered_dv_m_s", r.delivered_dv_m_s);
        d.set("equivalent_dv_m_s", r.equivalent_dv_m_s);
        d.set("lead_periods", r.lead_periods);
        if let Some(req) = r.required_dv_estimate_m_s {
            d.set("required_dv_m_s", req);
        }
        if let Some(margin) = r.margin {
            d.set("margin", margin);
        }
        d
    }

    /// Kick off one full-field tractor probe on a worker thread. `false` if one is
    /// already running, the scenario is not built, or the geometry is unflyable.
    ///
    /// **On demand, never per keystroke.** One probe is a full multi-year
    /// propagation — 12.4 s measured at the campaign's longest lead — which is
    /// precisely why the cheap [`tractor_readout`](Self::tractor_readout) exists
    /// beside it. Same bargain as the launch-window map's `[E]` verify.
    #[func]
    fn begin_tow_probe(
        &mut self,
        spacecraft_mass_kg: f64,
        hover_radii: f64,
        rock_radius_m: f64,
        lead_seconds: f64,
        duration_seconds: f64,
        retrograde: bool,
    ) -> bool {
        if self.tow_build.is_some() {
            return false;
        }
        let Some(core) = self.core.as_ref() else {
            self.error = "load() must succeed before begin_tow_probe()".into();
            return false;
        };
        let Some(scenario) = core.scenario_arc() else {
            self.error = "build the scenario before probing a tractor".into();
            return false;
        };
        let plan = TractorPlan {
            spacecraft_mass_kg,
            hover_radii,
            rock_radius_m,
            lead_seconds,
            duration_seconds,
            retrograde,
        };
        // Refuse rather than propagate: a spacecraft inside the asteroid has a
        // perfectly finite tow and no physical meaning, and 12 s of integration
        // would dignify it with a perigee.
        if !plan.is_flyable() {
            self.error =
                "the tractor would be inside the asteroid; raise the hover distance".into();
            return false;
        }
        // The stronger geometric refusal. A tow with no station-keeping solution
        // integrates perfectly happily and describes a mission nobody can fly, so
        // it is refused *by name* rather than being propagated for 12 s and then
        // shown beside a blank thrust.
        if !plan.holds_station() {
            self.error = "no station-keeping solution at that hover distance; the exhaust                           cant has passed 90 deg"
                .into();
            return false;
        }
        // A zero-length tow is a reachable knob setting (the duration control goes
        // to 0) and is not an error worth 12 s of integration or a raw
        // "invalid tow duration 0 s" from deep in the window constructor.
        if plan.effective_duration_seconds() <= 0.0 {
            self.error = "a tow of zero duration deflects nothing; raise the tow duration".into();
            return false;
        }
        if lead_seconds <= 0.0 {
            self.error = "a tractor needs a positive lead time".into();
            return false;
        }
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = probe_tow_plan(&scenario, &plan).map_err(|e| e.to_string());
            let _ = tx.send(result);
        });
        self.tow_build = Some(rx);
        // Drop the previous probe: one shown beside a running solve reads as this
        // solve's answer.
        self.pending_tow = Some(plan);
        self.tow_probe = None;
        self.error = GString::new();
        true
    }

    /// Whether a tractor probe is in flight.
    #[func]
    fn is_probing_tow(&self) -> bool {
        self.tow_build.is_some()
    }

    /// Pump the tractor-probe worker. `true` while **still running**, `false` once
    /// finished (or none in flight) — then [`tow_probe`](Self::tow_probe) holds
    /// the result. Non-blocking; safe every frame.
    #[func]
    fn poll_tow_probe(&mut self) -> bool {
        let Some(rx) = self.tow_build.as_ref() else {
            return false;
        };
        match rx.try_recv() {
            Err(mpsc::TryRecvError::Empty) => true,
            Ok(Ok((towed, nominal))) => {
                self.tow_build = None;
                if let Some(plan) = self.pending_tow.take() {
                    self.tow_probe = Some((plan, towed, nominal));
                }
                self.error = GString::new();
                false
            }
            Ok(Err(message)) => {
                self.tow_build = None;
                self.pending_tow = None;
                self.error = message.as_str().into();
                false
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.tow_build = None;
                self.pending_tow = None;
                self.error = "the tractor-probe thread died without reporting".into();
                false
            }
        }
    }

    /// The last full-field tractor probe, or an **empty dictionary** if none has
    /// been run against the current scenario.
    ///
    /// Keys: the six plan fields it was solved for (`spacecraft_mass_kg`,
    /// `hover_radii`, `rock_radius_m`, `lead_seconds`, `duration_seconds`,
    /// `retrograde`) so a display can tell "this readout's probe" from "a probe
    /// for knobs I have since moved"; plus `perigee_m`, `nominal_perigee_m`,
    /// `shift_m` and `clean_miss`.
    ///
    /// # `shift_m` is signed, and it must stay that way
    ///
    /// It is `perigee − nominal`, and a **negative value is a real, reachable
    /// outcome**: the nominal is a near-centre hit, so a tug too weak to carry the
    /// b-plane point past Earth walks it *toward* the centre first — the campaign
    /// measures −188 km for a 20 t tractor over the full lead. Reporting a
    /// magnitude here would hide the one result a user most needs to see, and
    /// would let a panel show steady progress while the impact deepened.
    ///
    /// `clean_miss` flags the pass that left the scan gate entirely, whose perigee
    /// is `+∞` — the success case that shares a sentinel with failure elsewhere in
    /// this codebase, so it is named rather than inferred from a magic number.
    #[func]
    fn tow_probe(&self) -> VarDictionary {
        let mut d = VarDictionary::new();
        let Some((plan, towed, nominal)) = self.tow_probe else {
            return d;
        };
        d.set("spacecraft_mass_kg", plan.spacecraft_mass_kg);
        d.set("hover_radii", plan.hover_radii);
        d.set("rock_radius_m", plan.rock_radius_m);
        d.set("lead_seconds", plan.lead_seconds);
        d.set("duration_seconds", plan.duration_seconds);
        d.set("retrograde", plan.retrograde);
        d.set("nominal_perigee_m", nominal);
        let clean = !towed.is_finite();
        d.set("clean_miss", clean);
        d.set("perigee_m", towed);
        // A clean miss has no finite perigee to difference against, so it carries
        // no shift rather than an infinite one.
        if !clean {
            d.set("shift_m", towed - nominal);
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

    /// Whether the most recent build attempt failed — the test a frontend should
    /// use after [`poll_build`](Self::poll_build) returns `false`.
    ///
    /// See the field's note for why `!is_ready()` is not that test: a failed
    /// *rebuild* leaves the previous threat installed and ready, so `is_ready()`
    /// reports success for a build that produced nothing.
    #[func]
    fn last_build_failed(&self) -> bool {
        self.build_failed
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

    /// [`body_position_ecl_au`](Self::body_position_ecl_au) for a whole list of
    /// NAIF ids at **one** epoch — one binding crossing instead of one per body.
    ///
    /// Returns a `PackedVector3Array` the same length and in the same order as
    /// `naif_ids`, with `Vector3::ZERO` wherever the single-body call would have
    /// returned `Vector3::ZERO`. **The contract is deliberately identical**,
    /// including the trap: ZERO in this heliocentric frame is *the Sun*, not a
    /// blank, so callers gate on `bodies_online` / the body's own span exactly as
    /// they always have. A batch that quietly dropped its misses would be worse
    /// than the trap — every body after a gap would take the previous one's
    /// position.
    ///
    /// **Why it exists.** The 3D view asks for twenty-five bodies at the clock
    /// every frame and paid twenty-five crossings for it, each marshalling two
    /// arguments and a `Vector3` back. The ephemeris work is unchanged (~11 µs a
    /// body either way); what this removes is the traffic, which `Sim.ffi_calls`
    /// counts and `tests/_perf.gd` prices.
    #[func]
    fn body_positions_ecl_au(
        &self,
        naif_ids: PackedInt64Array,
        tdb_seconds: f64,
    ) -> PackedVector3Array {
        let Some(core) = self.core.as_ref() else {
            // Not loaded: the same answer the singles give, one per id, so the
            // caller's zip against its own list still lines up.
            return PackedVector3Array::from(vec![Vector3::ZERO; naif_ids.len()]);
        };
        let ids: Vec<i32> = naif_ids.as_slice().iter().map(|&i| i as i32).collect();
        let out: Vec<Vector3> = core
            .body_positions_ecl_au(&ids, tdb_seconds)
            .into_iter()
            .map(|p| match p {
                Some(v) => Vector3::new(v.x as f32, v.y as f32, v.z as f32),
                None => Vector3::ZERO,
            })
            .collect();
        PackedVector3Array::from(out)
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
        if let Some((lo, hi)) = self
            .core
            .as_ref()
            .and_then(|c| c.encounter_sample_span_tdb())
        {
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
    /// Since the keyhole batch the sign and the axes are pinned (`B` points at the
    /// incoming asymptote; `ζ` opposes Earth's motion), so which side of the disc
    /// this lands on is now physics, not cosmetics — see
    /// [`bplane_frame_pinned`](Self::bplane_frame_pinned).
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

    /// Whether the encounter view's axes are the core's pinned Öpik `(ξ, ζ)`
    /// frame (`ξ` across Earth's motion, `ζ` against it). `false` before a build
    /// or on the display-basis fallback — label the frame accordingly.
    #[func]
    fn bplane_frame_pinned(&self) -> bool {
        self.core
            .as_ref()
            .map(|c| c.bplane_frame_pinned())
            .unwrap_or(false)
    }

    /// The resonant-return circles of the nominal encounter with returns of at
    /// most `max_years`, in the same `(ξ, ζ)` km frame as the tracks. Each entry:
    /// `h`, `k`, `a_prime_au`, `center_zeta_km`, `radius_km`, `b_min_km`,
    /// `b_max_km`, `crosses_capture_disc`, `near_xi_km`, `near_zeta_km`,
    /// `near_width_km`, `far_xi_km`, `far_zeta_km`, `far_width_km`. Empty before a
    /// build. Closed-form (microseconds) — safe to call on `plan_changed`.
    #[func]
    fn keyhole_circles(&self, max_years: i64) -> Array<VarDictionary> {
        let mut out = Array::new();
        let Some(core) = self.core.as_ref() else {
            return out;
        };
        for r in core.keyhole_circles(max_years.clamp(2, 20) as u32) {
            let mut d = VarDictionary::new();
            d.set("h", r.h as i64);
            d.set("k", r.k as i64);
            d.set("a_prime_au", r.a_prime_au);
            d.set("center_zeta_km", r.center_zeta_km);
            d.set("radius_km", r.radius_km);
            d.set("b_min_km", r.b_min_km);
            d.set("b_max_km", r.b_max_km);
            d.set("crosses_capture_disc", r.crosses_capture_disc);
            d.set("near_xi_km", r.near_point_km.0);
            d.set("near_zeta_km", r.near_point_km.1);
            d.set("near_width_km", r.near_width_km);
            d.set("far_xi_km", r.far_point_km.0);
            d.set("far_zeta_km", r.far_point_km.1);
            d.set("far_width_km", r.far_width_km);
            out.push(&d);
        }
        out
    }

    /// Where the current plan leaves the rock **in keyhole terms** — the planner's
    /// answer to *"a miss can be worse than a hit if it is the wrong miss"*.
    ///
    /// Empty dictionary when there is no plan, no pinned frame, or no b-plane
    /// reduction (a clean miss has left the 1.3 LD scan gate and has no b-point;
    /// the wide keyholes are the far ones, so such a pass flies past them
    /// unmeasured — say that rather than printing a blank).
    ///
    /// Otherwise: `plan_xi_km`, `plan_zeta_km`, `b_km`, `mapped_b_max_km`,
    /// `beyond_mapped_region`, and two sub-dictionaries — `nearest`, the closest
    /// *locus* in kilometres (the circle to name beside the drawn map), and
    /// `at_risk`, the closest *door edge* (the circle this plan is fewest
    /// kilometres from being inside, which is what an alert is cut on). Each row
    /// carries `h`, `k`, `a_prime_au`, `plan_a_prime_au`, `distance_km` (signed,
    /// + outside the circle), `width_km`, `widths_away`, `margin_km`, `inside`,
    /// `closest_xi_km`, `closest_zeta_km`.
    ///
    /// The second row was `tightest` — ranked by keyhole *widths* — until
    /// 2026-09-07. Five flown doors put the map's placement error at an additive
    /// 2 to 27 km that a width ratio divides away at a wide door, so the ranking
    /// moved to `margin_km` (`|distance| − width/2`) and the key changed name with
    /// it rather than quietly meaning something else.
    ///
    /// **The kilometres are a map coordinate, not a prediction of a return** —
    /// see `MissionCore::keyhole_readout`. Closed-form; safe on `plan_changed`.
    #[func]
    fn keyhole_readout(&self, max_years: i64) -> VarDictionary {
        let mut d = VarDictionary::new();
        let Some(r) = self
            .core
            .as_ref()
            .and_then(|c| c.keyhole_readout(max_years.clamp(2, 20) as u32))
        else {
            return d;
        };
        d.set("plan_xi_km", r.plan_point_km.0);
        d.set("plan_zeta_km", r.plan_point_km.1);
        d.set("b_km", r.b_km);
        d.set("mapped_b_max_km", r.mapped_b_max_km);
        d.set("beyond_mapped_region", r.beyond_mapped_region);
        d.set("nearest", &Self::keyhole_row(&r.nearest));
        d.set("at_risk", &Self::keyhole_row(&r.at_risk));
        d
    }

    /// One [`KeyholePlanRow`] as a dictionary — the two the readout returns share
    /// a shape so the panel can format either with one helper.
    fn keyhole_row(r: &KeyholePlanRow) -> VarDictionary {
        let mut d = VarDictionary::new();
        d.set("h", r.h as i64);
        d.set("k", r.k as i64);
        d.set("a_prime_au", r.a_prime_au);
        d.set("plan_a_prime_au", r.plan_a_prime_au);
        d.set("distance_km", r.distance_km);
        d.set("width_km", r.width_km);
        d.set("widths_away", r.widths_away);
        d.set("margin_km", r.margin_km);
        d.set("inside", r.inside);
        d.set("closest_xi_km", r.closest_point_km.0);
        d.set("closest_zeta_km", r.closest_point_km.1);
        d
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
