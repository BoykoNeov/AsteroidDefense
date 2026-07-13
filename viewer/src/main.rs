//! `viewer` — the egui frontend for the MVP (HANDOFF §1, §10 task 10 / §10.10).
//!
//! Two headline deliverables, side by side:
//!
//! 1. **The Δv-vs-lead-time curve** — how small an along-track nudge still turns
//!    the 2040 impact into a safe miss, as a function of how early it is applied.
//!    It is a *fixed* property of the designed impactor, so the `curve` binary
//!    computes it once (the ~680 s sweep) and writes `curve.json`; this app just
//!    loads and plots it (log-log, so the ~1/lead law reads as a straight line).
//!
//! 2. **The rewind→nudge→re-propagate animation** — pick a lead (rewind to a
//!    deflection epoch), dial in a Δv, and watch the encounter in Earth's frame:
//!    the nominal track spears Earth's capture disc (the hit), the deflected track
//!    slides past it (the miss). The displayed miss is the *same* validated
//!    b-plane perigee the curve solver uses ([`viewer::scenario::EncounterFrame`]).
//!
//! # Threading
//! Building the DE440 scenario takes ~10 s and each nudge re-propagates a short
//! arc (sub-second at ~1–2 orbits of lead — the `probe_prop` measurement). Both
//! run on a **worker thread**: the UI thread never blocks, and the worker calls
//! [`egui::Context::request_repaint`] when a frame lands so the view wakes. The
//! worker builds one [`DeflectionScenario`] and one nominal encounter up front and
//! reuses them per nudge (via [`RealFieldScenario::frame_from`]), so only the short
//! post-deflection arc is re-propagated — not the full nominal every time.
//!
//! Run (needs a display): `cargo run -p viewer --release` with the kernel env vars
//! set (`ASTEROID_DE_KERNEL` / `ASTEROID_PLANETARY_CONSTANTS`).

use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

use eframe::egui;
use egui::containers::{CentralPanel, Panel};
use egui::{Color32, Pos2, Sense, Shape, Stroke, Vec2};
use egui_plot::{Legend, Line, Plot, PlotPoints, Points, VLine};
use nalgebra::Vector3;

use asteroid_core::deflection::along_track_unit;
use viewer::scenario::{
    CurveFile, EncounterFrame, ImpactorConfig, RealFieldScenario, DEFAULT_CURVE_JSON,
    ENCOUNTER_HALF_WINDOW_SECONDS, ENCOUNTER_SAMPLES,
};

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1180.0, 720.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Asteroid Deflection Simulator",
        native_options,
        Box::new(|cc| Ok(Box::new(App::new(cc)) as Box<dyn eframe::App>)),
    )
}

// ---- Worker protocol --------------------------------------------------------

/// A nudge request: deflect `lead_seconds` before impact with an along-track Δv
/// of `dv_mag` m/s.
struct Request {
    lead_seconds: f64,
    dv_mag: f64,
}

/// Messages the worker sends back to the UI.
enum Msg {
    /// The scenario built; the campaign geometry the UI needs to scale controls.
    Ready {
        period_seconds: f64,
        max_lead_seconds: f64,
    },
    /// A computed encounter frame. Staleness is handled by the single-in-flight
    /// request discipline (the UI sends one nudge at a time and the worker
    /// coalesces to the newest queued one), so the frame needs no request tag.
    Frame(Box<EncounterFrame>),
    /// The build or a nudge failed (e.g. missing kernel env vars).
    Error(String),
}

/// Build the scenario once, then serve nudge requests, coalescing to the latest
/// queued one so a fast slider drag does not back up a queue of stale props.
fn worker_main(ctx: egui::Context, req_rx: Receiver<Request>, msg_tx: Sender<Msg>) {
    let cfg = ImpactorConfig::default();
    let sc = match RealFieldScenario::build(&cfg) {
        Ok(s) => s,
        Err(e) => {
            let _ = msg_tx.send(Msg::Error(format!("scenario build failed: {e}")));
            ctx.request_repaint();
            return;
        }
    };
    // One DeflectionScenario (the expensive full-nominal propagation) and one
    // nominal encounter (the full-span scan) — both done once, reused per nudge.
    let ds = match sc.deflection() {
        Ok(d) => d,
        Err(e) => {
            let _ = msg_tx.send(Msg::Error(format!("deflection scenario failed: {e}")));
            ctx.request_repaint();
            return;
        }
    };
    let nominal_enc = match sc.nominal_hit(&ds) {
        Ok(e) => e,
        Err(e) => {
            let _ = msg_tx.send(Msg::Error(format!("nominal encounter failed: {e}")));
            ctx.request_repaint();
            return;
        }
    };

    let max_lead =
        sc.impact_epoch().tdb_seconds_past_j2000() - sc.epoch0().tdb_seconds_past_j2000();
    let _ = msg_tx.send(Msg::Ready {
        period_seconds: sc.period_seconds,
        max_lead_seconds: max_lead,
    });
    ctx.request_repaint();

    while let Ok(mut req) = req_rx.recv() {
        // Coalesce: skip to the newest queued request.
        while let Ok(next) = req_rx.try_recv() {
            req = next;
        }

        let defl_epoch = sc.impact_epoch().shifted_by_seconds(-req.lead_seconds);
        // Along-track heading from the nominal state at the deflection epoch.
        let dv = match ds.nominal().state_at(defl_epoch) {
            Ok(state) => match along_track_unit(state) {
                Some(dir) => req.dv_mag * dir,
                None => Vector3::zeros(),
            },
            Err(_) => Vector3::zeros(),
        };

        match sc.frame_from(
            &ds,
            nominal_enc,
            defl_epoch,
            dv,
            ENCOUNTER_HALF_WINDOW_SECONDS,
            ENCOUNTER_SAMPLES,
        ) {
            Ok(frame) => {
                let _ = msg_tx.send(Msg::Frame(Box::new(frame)));
            }
            Err(e) => {
                let _ = msg_tx.send(Msg::Error(format!("nudge failed: {e}")));
            }
        }
        ctx.request_repaint();
    }
}

// ---- App --------------------------------------------------------------------

enum Status {
    Building,
    Ready,
    Failed(String),
}

struct App {
    req_tx: Sender<Request>,
    msg_rx: Receiver<Msg>,
    status: Status,

    /// Heliocentric period, seconds (from `Ready`) — the lead-slider unit.
    period_seconds: f64,
    /// Longest lead the campaign supports, seconds (from `Ready`).
    max_lead_seconds: f64,

    /// Current control values.
    lead_orbits: f64,
    dv_mag: f64,

    /// The last (lead_seconds, dv_mag) actually sent to the worker, and whether a
    /// request is in flight (so drags coalesce to ~one outstanding prop).
    requested: Option<(f64, f64)>,
    in_flight: bool,

    /// The most recent computed frame and the params it was for.
    frame: Option<EncounterFrame>,

    /// The cached headline curve (`curve.json`), if present.
    curve: Option<CurveFile>,
    curve_note: String,

    /// Animation cursor along the encounter window, in [0, 1], and whether it is
    /// advancing.
    playing: bool,
    anim_phase: f64,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let (req_tx, req_rx) = mpsc::channel::<Request>();
        let (msg_tx, msg_rx) = mpsc::channel::<Msg>();
        let ctx = cc.egui_ctx.clone();
        thread::spawn(move || worker_main(ctx, req_rx, msg_tx));

        let (curve, curve_note) = match std::fs::read_to_string(DEFAULT_CURVE_JSON) {
            Ok(s) => match serde_json::from_str::<CurveFile>(&s) {
                Ok(c) => {
                    let n = c.points.len();
                    (Some(c), format!("loaded {n} points from {DEFAULT_CURVE_JSON}"))
                }
                Err(e) => (None, format!("{DEFAULT_CURVE_JSON} is unreadable: {e}")),
            },
            Err(_) => (
                None,
                format!(
                    "no {DEFAULT_CURVE_JSON} — run `cargo run -p viewer --bin curve` to generate it"
                ),
            ),
        };

        Self {
            req_tx,
            msg_rx,
            status: Status::Building,
            period_seconds: 0.0,
            max_lead_seconds: 0.0,
            lead_orbits: 1.5,
            dv_mag: 0.0,
            requested: None,
            in_flight: false,
            frame: None,
            curve,
            curve_note,
            playing: false,
            anim_phase: 0.0,
        }
    }

    /// Current lead in seconds, clamped to what the campaign supports.
    fn lead_seconds(&self) -> f64 {
        (self.lead_orbits * self.period_seconds).min(self.max_lead_seconds)
    }

    /// The required Δv at the current lead, linearly interpolated from the cached
    /// curve (in lead-periods), or `None` if there is no curve / too few points.
    fn suggested_dv(&self) -> Option<f64> {
        let pts = &self.curve.as_ref()?.points;
        if pts.len() < 2 {
            return None;
        }
        let x = self.lead_orbits;
        if x <= pts[0].lead_periods {
            return Some(pts[0].required_dv);
        }
        for w in pts.windows(2) {
            let (a, b) = (&w[0], &w[1]);
            if x >= a.lead_periods && x <= b.lead_periods {
                let t = (x - a.lead_periods) / (b.lead_periods - a.lead_periods);
                return Some(a.required_dv + t * (b.required_dv - a.required_dv));
            }
        }
        Some(pts[pts.len() - 1].required_dv)
    }

    /// Drain worker messages into UI state.
    fn pump(&mut self) {
        while let Ok(msg) = self.msg_rx.try_recv() {
            match msg {
                Msg::Ready {
                    period_seconds,
                    max_lead_seconds,
                } => {
                    self.period_seconds = period_seconds;
                    self.max_lead_seconds = max_lead_seconds;
                    self.status = Status::Ready;
                }
                Msg::Frame(frame) => {
                    self.frame = Some(*frame);
                    self.in_flight = false;
                }
                Msg::Error(e) => {
                    self.status = Status::Failed(e);
                    self.in_flight = false;
                }
            }
        }
    }

    /// If the controls have moved since the last request and nothing is in flight,
    /// ask the worker for a fresh frame.
    fn maybe_request(&mut self) {
        if !matches!(self.status, Status::Ready) {
            return;
        }
        let want = (self.lead_seconds(), self.dv_mag);
        if self.in_flight || self.requested == Some(want) {
            return;
        }
        if self
            .req_tx
            .send(Request {
                lead_seconds: want.0,
                dv_mag: want.1,
            })
            .is_ok()
        {
            self.requested = Some(want);
            self.in_flight = true;
        }
    }
}

impl eframe::App for App {
    // eframe 0.35 hands the app a root `Ui` (not a `Context`); panels attach with
    // the Ui-based `show`. The `Context` (for cross-thread repaint and frame-time)
    // comes off the ui.
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.pump();
        self.maybe_request();

        Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Asteroid Deflection Simulator");
                ui.separator();
                match &self.status {
                    Status::Building => {
                        ui.spinner();
                        ui.label("building DE440 scenario (~10 s)…");
                    }
                    Status::Ready => {
                        ui.label("ready");
                        if self.in_flight {
                            ui.spinner();
                            ui.label("propagating nudge…");
                        }
                    }
                    Status::Failed(e) => {
                        ui.colored_label(Color32::LIGHT_RED, format!("error: {e}"));
                    }
                }
            });
        });

        Panel::left("controls")
            .resizable(false)
            .default_size(430.0)
            .show(ui, |ui| {
                self.controls_and_curve(ui);
            });

        CentralPanel::default().show(ui, |ui| {
            self.encounter_view(ui, &ctx);
        });
    }
}

impl App {
    fn controls_and_curve(&mut self, ui: &mut egui::Ui) {
        let ready = matches!(self.status, Status::Ready);

        ui.add_space(6.0);
        ui.label("Rewind — how early do you deflect?");
        ui.add_enabled(
            ready,
            egui::Slider::new(&mut self.lead_orbits, 0.5..=8.0).text("lead (orbits)"),
        );
        if ready {
            let yr = self.lead_seconds() / (365.25 * 86_400.0);
            ui.label(format!("  = {yr:.2} yr before impact"));
        }

        ui.add_space(10.0);
        ui.label("Nudge — along-track Δv");
        ui.add_enabled(
            ready,
            egui::Slider::new(&mut self.dv_mag, 0.0..=3.0).text("Δv (m/s)"),
        );
        if let Some(dv) = self.suggested_dv() {
            ui.horizontal(|ui| {
                ui.label(format!("required here ≈ {dv:.3} m/s"));
                if ui.add_enabled(ready, egui::Button::new("use")).clicked() {
                    self.dv_mag = dv.clamp(0.0, 3.0);
                }
            });
        }

        ui.add_space(10.0);
        ui.horizontal(|ui| {
            let label = if self.playing { "⏸ pause" } else { "▶ play" };
            if ui
                .add_enabled(self.frame.is_some(), egui::Button::new(label))
                .clicked()
            {
                self.playing = !self.playing;
                if self.anim_phase >= 1.0 {
                    self.anim_phase = 0.0;
                }
            }
            if ui.button("⟲ reset").clicked() {
                self.anim_phase = 0.0;
            }
        });

        // The current encounter outcome, straight from the frame's b-plane numbers.
        if let Some(f) = &self.frame {
            ui.add_space(6.0);
            ui.label(format!("capture radius: {:.0} km", f.capture_radius / 1000.0));
            match f.deflected_perigee {
                Some(p) if p <= f.capture_radius => {
                    ui.colored_label(
                        Color32::LIGHT_RED,
                        format!("IMPACT — perigee {:.0} km (inside capture)", p / 1000.0),
                    );
                }
                Some(p) => {
                    ui.colored_label(
                        Color32::LIGHT_GREEN,
                        format!("MISS — perigee {:.0} km", p / 1000.0),
                    );
                }
                None => {
                    ui.colored_label(Color32::LIGHT_GREEN, "clean MISS — left the scan window");
                }
            }
        }

        ui.add_space(14.0);
        ui.separator();
        ui.label("Headline curve: required Δv vs lead time");
        ui.small(&self.curve_note);
        self.curve_plot(ui);
    }

    fn curve_plot(&self, ui: &mut egui::Ui) {
        let Some(curve) = &self.curve else {
            ui.add_space(8.0);
            ui.weak("(plot appears once curve.json exists)");
            return;
        };

        // Log-log: the Δv ∝ 1/lead law is a straight line of slope ≈ −1. Plot
        // log10(lead in orbits) vs log10(Δv in mm/s).
        let pts: Vec<[f64; 2]> = curve
            .points
            .iter()
            .filter(|p| p.lead_periods > 0.0 && p.required_dv > 0.0)
            .map(|p| [p.lead_periods.log10(), (p.required_dv * 1000.0).log10()])
            .collect();

        let marker_x = self.lead_orbits.max(1e-6).log10();

        Plot::new("dv_curve")
            .view_aspect(1.4)
            .legend(Legend::default())
            .x_axis_label("log₁₀ lead (orbits)")
            .y_axis_label("log₁₀ Δv (mm/s)")
            .show(ui, |pui| {
                pui.line(Line::new("required Δv", PlotPoints::from(pts.clone())));
                pui.points(
                    Points::new("", PlotPoints::from(pts))
                        .radius(3.0)
                        .color(Color32::from_rgb(120, 170, 255)),
                );
                pui.vline(VLine::new("current lead", marker_x).color(Color32::YELLOW));
            });
    }

    fn encounter_view(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.label(
            "Encounter in Earth's frame — nominal (red) spears the disc; deflected (green) slides past",
        );

        let Some(frame) = &self.frame else {
            ui.centered_and_justified(|ui| {
                ui.weak("waiting for the first propagation…");
            });
            return;
        };
        if frame.nominal.len() < 2 {
            return;
        }

        // Advance the animation cursor (~6 s to sweep the whole window).
        if self.playing {
            let dt = ctx.input(|i| i.stable_dt) as f64;
            self.anim_phase += dt / 6.0;
            if self.anim_phase >= 1.0 {
                self.anim_phase = 1.0;
                self.playing = false;
            }
            ctx.request_repaint();
        }

        // 2D projection basis from the nominal track: e1 along the flight
        // direction, e2 perpendicular, pointing toward the closest pass.
        let (e1, e2) = projection_basis(frame);

        // World half-extent: fit the capture disc and the (possibly wider)
        // deflected miss, with margin.
        let miss = frame
            .deflected_perigee
            .unwrap_or(frame.capture_radius)
            .max(frame.nominal_perigee);
        let half_extent = (frame.capture_radius.max(miss) * 2.6).max(frame.earth_radius * 3.0);

        let avail = ui.available_size();
        let side = avail.x.min(avail.y).max(200.0);
        let (resp, painter) = ui.allocate_painter(Vec2::splat(side), Sense::hover());
        let rect = resp.rect;
        let center = rect.center();
        let scale = (side as f64 * 0.5) / half_extent; // world m → px

        let to_screen = |p2: [f64; 2]| -> Pos2 {
            Pos2::new(
                center.x + (p2[0] * scale) as f32,
                center.y - (p2[1] * scale) as f32,
            )
        };
        let project = |v: &Vector3<f64>| -> [f64; 2] { [v.dot(&e1), v.dot(&e2)] };

        // Earth's focused capture disc (the collision cross-section) and the solid
        // body inside it.
        painter.circle_stroke(
            center,
            (frame.capture_radius * scale) as f32,
            Stroke::new(1.5, Color32::from_rgb(230, 160, 60)),
        );
        painter.circle_filled(
            center,
            (frame.earth_radius * scale) as f32,
            Color32::from_rgb(70, 120, 210),
        );

        // The two tracks.
        draw_track(
            &painter,
            &frame.nominal,
            &to_screen,
            &project,
            Color32::from_rgb(230, 90, 90),
        );
        draw_track(
            &painter,
            &frame.deflected,
            &to_screen,
            &project,
            Color32::from_rgb(90, 210, 120),
        );

        // The moving asteroids at the animation cursor. At 18 km/s the pass crosses
        // this ~50,000 km box in ~1.6 h — barely 2% of the ±1.5-day sample window —
        // so sweeping the cursor over the *whole* window would leave both dots
        // clipped off-screen for almost the entire play. Instead sweep only the
        // contiguous band of samples that projects inside the frame (union of both
        // tracks, with a little margin so they enter and exit visibly).
        let margin = half_extent * 1.15;
        let in_box = |v: &Vector3<f64>| -> bool {
            let p = project(v);
            p[0].abs() <= margin && p[1].abs() <= margin
        };
        let mut lo = frame.nominal.len();
        let mut hi = 0usize;
        for i in 0..frame.nominal.len() {
            if in_box(&frame.nominal[i]) || in_box(&frame.deflected[i]) {
                lo = lo.min(i);
                hi = hi.max(i);
            }
        }
        let (lo, hi) = if lo <= hi {
            (lo, hi)
        } else {
            (0, frame.nominal.len() - 1) // never in box (shouldn't happen): whole span
        };
        let span = (hi - lo).max(1) as f64;
        let idx = (lo + (self.anim_phase * span).round() as usize).min(frame.nominal.len() - 1);
        painter.circle_filled(
            to_screen(project(&frame.nominal[idx])),
            4.0,
            Color32::from_rgb(230, 90, 90),
        );
        painter.circle_filled(
            to_screen(project(&frame.deflected[idx])),
            4.0,
            Color32::from_rgb(90, 210, 120),
        );

        // A scale bar (10 000 km) so the miss reads as a distance, not a picture.
        let bar_px = (1.0e7 * scale) as f32;
        let y = rect.bottom() - 18.0;
        let x0 = rect.left() + 16.0;
        painter.line_segment(
            [Pos2::new(x0, y), Pos2::new(x0 + bar_px, y)],
            Stroke::new(2.0, Color32::GRAY),
        );
        painter.text(
            Pos2::new(x0, y - 14.0),
            egui::Align2::LEFT_CENTER,
            "10 000 km",
            egui::FontId::proportional(12.0),
            Color32::GRAY,
        );
    }
}

/// A 2D basis for projecting the 3D geocentric tracks: `e1` along the nominal
/// flight direction (its chord across the window), `e2` perpendicular to `e1` in
/// the plane containing the closest approach — so the incoming/outgoing legs run
/// horizontally and the miss opens vertically.
fn projection_basis(frame: &EncounterFrame) -> (Vector3<f64>, Vector3<f64>) {
    let n = frame.nominal.len();
    let chord = frame.nominal[n - 1] - frame.nominal[0];
    let e1 = if chord.norm() > 0.0 {
        chord.normalize()
    } else {
        Vector3::x()
    };
    // Closest nominal sample to Earth (the perigee point).
    let close = frame
        .nominal
        .iter()
        .min_by(|a, b| a.norm().partial_cmp(&b.norm()).unwrap())
        .copied()
        .unwrap_or_else(Vector3::y);
    let perp = close - close.dot(&e1) * e1;
    let e2 = if perp.norm() > 0.0 {
        perp.normalize()
    } else {
        // Degenerate (closest point on the chord line): any perpendicular will do.
        let a = if e1.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        (a - a.dot(&e1) * e1).normalize()
    };
    (e1, e2)
}

/// Draw a geocentric track as a projected polyline.
fn draw_track(
    painter: &egui::Painter,
    track: &[Vector3<f64>],
    to_screen: &impl Fn([f64; 2]) -> Pos2,
    project: &impl Fn(&Vector3<f64>) -> [f64; 2],
    color: Color32,
) {
    let pts: Vec<Pos2> = track.iter().map(|v| to_screen(project(v))).collect();
    painter.add(Shape::line(pts, Stroke::new(1.5, color)));
}
