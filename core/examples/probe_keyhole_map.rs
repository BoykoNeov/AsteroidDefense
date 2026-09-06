//! The b-plane keyhole map of the shipping encounter, as data: the capture disc,
//! the Tier-3 uncertainty ellipse, every reachable resonant-return circle with its
//! keyhole widths, and the nominal and two deflected b-points — all in the pinned
//! Öpik `(ξ, ζ)` frame of `core/src/keyhole.rs`.
//!
//! This is the picture the two pricing probes (`probe_keyhole_reach`,
//! `probe_keyhole_rotation`) were sketching numerically, drawn from the closed
//! form instead of a direction sweep. It exists to be *rendered*
//! (`tools/keyhole_map_svg.py`) and to feed the frontend the same numbers, so the
//! JSON it writes is deliberately plain: no serde in core, every field named, SI
//! converted to km at this boundary.
//!
//! What it prints and what would falsify it:
//!
//!   1. The round-trip gate (`incoming_semi_major_axis` against the real
//!      pre-encounter orbit) — a frame or sign bug shows up here at the percent
//!      level, and the probe exits non-zero rather than draw a wrong map.
//!   2. The ellipse, transformed from the sensitivity's arbitrary basis into ξ,ζ.
//!      The transformation is a rotation, so its σ-axes must be unchanged by it.
//!   3. Two deflected passes flown in the real field — a prograde and a retrograde
//!      along-track nudge at the campaign start — and where the *closed form* says
//!      their outgoing `a'` is, against the vis-viva read 30 days after each flown
//!      encounter. This is the rotation probe's check repeated on both ζ sides
//!      for the cost of two propagations instead of a 190 s Δv solve.
//!
//! Requires kernels. ~50 s (build, the 13-propagation sensitivity, two deflected
//! re-flies).
//!
//!   cargo run -p asteroid_core --release --example probe_keyhole_map -- [out.json]

use anise::constants::frames::{EARTH_J2000, SSB_J2000, SUN_J2000};
use asteroid_core::{
    along_track_unit, closest_approach, EphemerisPerturber, Epoch, ImpactorConfig, OpikFrame,
    RealFieldScenario, ResonantCircle, ScanOptions, StateCovariance, AU_M,
};
use nalgebra::{Vector2, Vector3};
use std::fmt::Write as _;
use std::time::Instant;

/// The scan gate the shipping scenario uses — mirrored, as the other probes do.
const SHIPPING_SCAN_GATE_M: f64 = 5.0e8;
/// The along-track nudge flown each way, m/s. `probe_keyhole_rotation` measured
/// 0.205 m/s for a 21 R⊕ miss at this lead; 0.2 lands near that.
const NUDGE_M_S: f64 = 0.2;
/// Resonant-return window and revolution cap for the census.
const RETURN_YEARS: std::ops::RangeInclusive<u32> = 2..=20;
const MAX_REVOLUTIONS: u32 = 24;
/// Circles farther out than this (in capture radii) are omitted from the map.
const B_MAX_CAPTURE_RADII: f64 = 60.0;

fn semi_major(r: &Vector3<f64>, v: &Vector3<f64>, mu: f64) -> f64 {
    1.0 / (2.0 / r.norm() - v.norm_squared() / mu)
}

fn json_vec2(v: Vector2<f64>, scale: f64) -> String {
    format!("[{:.6}, {:.6}]", v.x * scale, v.y * scale)
}

fn json_vec3(v: Vector3<f64>) -> String {
    format!("[{:.12}, {:.12}, {:.12}]", v.x, v.y, v.z)
}

struct Deflected {
    label: &'static str,
    dv: f64,
    point: Vector2<f64>,
    b: f64,
    perigee: f64,
    hit: bool,
    a_closed_form: f64,
    a_flown: Option<f64>,
}

fn main() {
    let out_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "docs/keyhole_map.json".to_string());

    let cfg = ImpactorConfig::default();
    let t = Instant::now();
    let scenario = match RealFieldScenario::build(&cfg) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("build failed: {e}");
            eprintln!("(this probe needs DE440 kernels — see kernels::resolve)");
            std::process::exit(1);
        }
    };
    println!("build: {:.2} s", t.elapsed().as_secs_f64());

    let eph = scenario.ephemeris().clone();
    let mu_sun = eph.sun_gm_m3_s2().expect("sun GM");
    let earth = EphemerisPerturber::new(eph.clone(), EARTH_J2000);
    let scan = ScanOptions {
        max_sample_dt: 6.0 * 3600.0,
        time_tol_seconds: 1.0e-3,
        max_distance: Some(SHIPPING_SCAN_GATE_M),
    };
    let ds = scenario.deflection().expect("deflection");
    let t_ca = ds
        .nominal_encounter_epoch()
        .expect("encounter epoch")
        .expect("a close approach inside the scan gate");

    // ---- the sensitivity, and the frame built from ITS reduction ---------------
    //
    // The ellipse lives in the fixed-epoch reduction's basis. Building the Öpik
    // frame from that same reduction makes the rotation between the two frames
    // exact (same Ŝ) instead of near-exact (CA Ŝ versus fixed-epoch Ŝ, which agree
    // to the asymptotic invariance and no better).
    let t = Instant::now();
    let sens = scenario.bplane_sensitivity().expect("bplane sensitivity");
    println!("sensitivity: {:.1} s", t.elapsed().as_secs_f64());
    let enc = sens.nominal;
    let (r_e_km, v_e_km) = eph
        .state_km_s(EARTH_J2000, SUN_J2000, t_ca.as_hifitime())
        .expect("Earth heliocentric state at encounter");
    let frame = OpikFrame::new(&enc, r_e_km * 1e3, v_e_km * 1e3, mu_sun).expect("Öpik frame");

    // ---- check 1: the round trip ---------------------------------------------
    let t_pre = t_ca.shifted_by_seconds(-30.0 * 86_400.0);
    let helio_a = |clock: &asteroid_core::Clock, epoch: Epoch| -> Option<f64> {
        let st = clock.state_at(epoch).ok()?;
        let (rs, vs) = eph
            .state_km_s(SUN_J2000, SSB_J2000, epoch.as_hifitime())
            .ok()?;
        Some(semi_major(
            &(st.position - rs * 1e3),
            &(st.velocity - vs * 1e3),
            mu_sun,
        ))
    };
    let a_actual = helio_a(ds.nominal(), t_pre).expect("pre-encounter state");
    let a_in = frame.incoming_semi_major_axis();
    let round_trip = (a_in - a_actual).abs() / a_actual;
    println!(
        "round trip: reconstructed {:.9} AU vs actual {:.9} AU ({round_trip:.3e})",
        a_in / AU_M,
        a_actual / AU_M
    );
    if round_trip > 1.0e-3 {
        eprintln!("*** round trip failed: frame/sign/vis-viva bug, not the approximation");
        std::process::exit(1);
    }

    // ---- check 2: the ellipse in ξ,ζ ---------------------------------------
    let seed = ds
        .nominal()
        .state_at(scenario.epoch0())
        .expect("seed at epoch0");
    let cov = StateCovariance::synthetic_along_track(seed, 5.0e-5, 20.0, 1.0e3)
        .expect("non-degenerate seed");
    let unc = sens.map(&cov);
    // Through the basis' own method since 2026-09-06, not six lines here. The
    // gdext binding's `Tier3View` needs exactly this rotation to draw the ellipse
    // on the b-plane view's axes, and two copies of it would be two pictures that
    // must agree with no way to check.
    let (rot, orth) = sens.basis.rotation_to(frame.xi_hat, frame.zeta_hat);
    if orth > 1e-9 {
        eprintln!("*** basis change is not orthonormal ({orth:.3e}): the two frames share no Ŝ");
        std::process::exit(1);
    }
    let mean_xz = rot * unc.mean;
    let cov_xz = rot * unc.covariance * rot.transpose();
    let (sig_major, sig_minor) = unc.sigma_axes();
    let eig = cov_xz.symmetric_eigen();
    let (i_major, _) = if eig.eigenvalues[0] >= eig.eigenvalues[1] {
        (0, 1)
    } else {
        (1, 0)
    };
    let major_axis = eig.eigenvectors.column(i_major);
    let major_angle_deg = major_axis[1].atan2(major_axis[0]).to_degrees();
    let p_impact = unc.impact_probability().expect("impact probability");
    let nominal_xz = frame.project(&enc.b_vector);
    println!(
        "ellipse: σ {:.3} × {:.3} km, major axis {:.1}° from ξ, mean ({:.1}, {:.1}) km, P(impact) {:.4}",
        sig_major / 1e3,
        sig_minor / 1e3,
        major_angle_deg,
        mean_xz.x / 1e3,
        mean_xz.y / 1e3,
        p_impact
    );
    println!(
        "nominal b-point: ξ {:.1} km, ζ {:.1} km  (|B| {:.1} km, capture {:.1} km)",
        nominal_xz.x / 1e3,
        nominal_xz.y / 1e3,
        enc.impact_parameter / 1e3,
        enc.capture_radius / 1e3
    );

    // ---- check 3: two deflected passes, both ζ sides --------------------------
    let epoch0 = scenario.epoch0();
    let direction = along_track_unit(seed).expect("non-degenerate seed velocity");
    let mut deflected = Vec::new();
    for (label, dv) in [("prograde", NUDGE_M_S), ("retrograde", -NUDGE_M_S)] {
        let t = Instant::now();
        let (clock, _) = ds
            .deflected_trajectory(epoch0, dv * direction)
            .expect("deflected trajectory");
        let ca = closest_approach(&clock, &earth, scan)
            .expect("scan the deflected arc")
            .expect("the deflected pass is still inside the gate");
        let d_enc = ca
            .b_plane(enc.mu, enc.earth_radius)
            .expect("reduce the deflected encounter");
        // Its own b-vector, expressed in the nominal frame, rescaled to its own
        // |B| (the deflected asymptote differs from the nominal's by ~0.1°).
        let raw = frame.project(&d_enc.b_vector);
        let point = raw * (d_enc.impact_parameter / raw.norm());
        let a_closed_form = frame.post_encounter_semi_major_axis(point);
        let a_flown = [45.0, 30.0, 20.0]
            .iter()
            .find_map(|d| helio_a(&clock, ca.epoch.shifted_by_seconds(d * 86_400.0)));
        println!(
            "{label} {dv:+.3} m/s at epoch0 → b {:.0} km, perigee {:.0} km ({:.2} R⊕), hit {}; \
             ξ {:.0} ζ {:.0} km; a' closed form {:.6} AU, flown {}  [{:.1} s]",
            d_enc.impact_parameter / 1e3,
            d_enc.perigee / 1e3,
            d_enc.perigee / enc.earth_radius,
            d_enc.is_hit(),
            point.x / 1e3,
            point.y / 1e3,
            a_closed_form / AU_M,
            a_flown
                .map(|a| format!(
                    "{:.6} AU (rel {:.2e})",
                    a / AU_M,
                    (a - a_closed_form).abs() / a
                ))
                .unwrap_or_else(|| "n/a".into()),
            t.elapsed().as_secs_f64()
        );
        deflected.push(Deflected {
            label,
            dv,
            point,
            b: d_enc.impact_parameter,
            perigee: d_enc.perigee,
            hit: d_enc.is_hit(),
            a_closed_form,
            a_flown,
        });
    }

    // ---- the circles ---------------------------------------------------------
    let b_max = B_MAX_CAPTURE_RADII * frame.capture_radius;
    let circles: Vec<ResonantCircle> = frame.resonant_circles(RETURN_YEARS, MAX_REVOLUTIONS, b_max);
    println!(
        "\n{} resonant circles within {B_MAX_CAPTURE_RADII:.0} capture radii (a' in \
         {:.4}..{:.4} AU reach):",
        circles.len(),
        circles
            .iter()
            .map(|c| c.a_prime)
            .fold(f64::INFINITY, f64::min)
            / AU_M,
        circles
            .iter()
            .map(|c| c.a_prime)
            .fold(f64::NEG_INFINITY, f64::max)
            / AU_M
    );
    for c in circles.iter().filter(|c| c.resonance.h <= 7) {
        let (lo, hi) = c.b_range();
        let graze = c.intersections_at_radius(frame.capture_radius);
        let w_near = graze
            .map(|(g, _)| frame.keyhole_at(c, g).width)
            .unwrap_or_else(|| frame.keyhole_at(c, c.nearest_point()).width);
        let w_far = frame.keyhole_at(c, c.farthest_point()).width;
        println!(
            "  {:>5}  a' {:.6} AU  ζ_c {:>9.0} km  R {:>9.0} km  b {:>7.0}..{:>7.0} km  {}  \
             keyhole {:.2}..{:.2} km",
            c.resonance.to_string(),
            c.a_prime / AU_M,
            c.center_zeta / 1e3,
            c.radius / 1e3,
            lo / 1e3,
            hi / 1e3,
            if c.crosses_capture_disc(frame.capture_radius) {
                "through disc"
            } else {
                "clear       "
            },
            w_near / 1e3,
            w_far / 1e3
        );
    }

    // ---- JSON ----------------------------------------------------------------
    let km = 1.0e-3;
    let mut j = String::new();
    j.push_str("{\n");
    let _ = writeln!(j, "  \"schema\": \"asteroid-keyhole-map/1\",");
    let _ = writeln!(j, "  \"encounter_epoch_tdb\": \"{}\",", t_ca.as_hifitime());
    let _ = writeln!(
        j,
        "  \"units\": {{\"length\": \"km\", \"speed\": \"km/s\", \"angle\": \"deg\"}},"
    );
    let _ = writeln!(
        j,
        "  \"frame\": {{\"xi_hat_icrf\": {}, \"zeta_hat_icrf\": {}, \"eta_hat_icrf\": {}, \
         \"theta_deg\": {:.6}, \"c_km\": {:.6}}},",
        json_vec3(frame.xi_hat),
        json_vec3(frame.zeta_hat),
        json_vec3(frame.eta_hat),
        frame.theta().to_degrees(),
        frame.c() * km
    );
    let _ = writeln!(
        j,
        "  \"encounter\": {{\"v_inf_km_s\": {:.6}, \"capture_radius_km\": {:.3}, \
         \"earth_radius_km\": {:.3}, \"incoming_a_au\": {:.9}, \"round_trip_rel\": {:.3e}, \
         \"earth_speed_km_s\": {:.6}}},",
        frame.v_inf * km,
        frame.capture_radius * km,
        enc.earth_radius * km,
        a_in / AU_M,
        round_trip,
        frame.v_earth.norm() * km
    );
    let _ = writeln!(
        j,
        "  \"nominal\": {{\"point_km\": {}, \"b_km\": {:.3}, \"perigee_km\": {:.3}, \"hit\": {}}},",
        json_vec2(nominal_xz, km),
        enc.impact_parameter * km,
        enc.perigee * km,
        enc.is_hit()
    );
    j.push_str("  \"deflected\": [\n");
    for (i, d) in deflected.iter().enumerate() {
        let _ = writeln!(
            j,
            "    {{\"label\": \"{}\", \"dv_m_s\": {:.3}, \"point_km\": {}, \"b_km\": {:.3}, \
             \"perigee_km\": {:.3}, \"hit\": {}, \"a_closed_form_au\": {:.9}, \"a_flown_au\": {}}}{}",
            d.label,
            d.dv,
            json_vec2(d.point, km),
            d.b * km,
            d.perigee * km,
            d.hit,
            d.a_closed_form / AU_M,
            d.a_flown
                .map(|a| format!("{:.9}", a / AU_M))
                .unwrap_or_else(|| "null".into()),
            if i + 1 < deflected.len() { "," } else { "" }
        );
    }
    j.push_str("  ],\n");
    let _ = writeln!(
        j,
        "  \"ellipse\": {{\"label\": \"invented along-track covariance (σ_v 5e-5 m/s, 20:1, \
         1 km)\", \"mean_km\": {}, \"cov_km2\": [[{:.9}, {:.9}], [{:.9}, {:.9}]], \
         \"sigma_axes_km\": [{:.6}, {:.6}], \"major_axis_deg_from_xi\": {:.3}, \
         \"impact_probability\": {:.6}}},",
        json_vec2(mean_xz, km),
        cov_xz[(0, 0)] * km * km,
        cov_xz[(0, 1)] * km * km,
        cov_xz[(1, 0)] * km * km,
        cov_xz[(1, 1)] * km * km,
        sig_major * km,
        sig_minor * km,
        major_angle_deg,
        p_impact
    );
    j.push_str("  \"circles\": [\n");
    for (i, c) in circles.iter().enumerate() {
        let (lo, hi) = c.b_range();
        let graze = c.intersections_at_radius(frame.capture_radius);
        let far = frame.keyhole_at(c, c.farthest_point());
        let near_pt = c.nearest_point();
        let near = frame.keyhole_at(c, near_pt);
        let graze_json = match graze {
            Some((g, _)) => {
                let k = frame.keyhole_at(c, g);
                format!(
                    "{{\"point_km\": {}, \"width_km\": {:.6}}}",
                    json_vec2(g, km),
                    k.width * km
                )
            }
            None => "null".to_string(),
        };
        let _ = writeln!(
            j,
            "    {{\"h\": {}, \"k\": {}, \"a_prime_au\": {:.9}, \"center_zeta_km\": {:.3}, \
             \"radius_km\": {:.3}, \"b_min_km\": {:.3}, \"b_max_km\": {:.3}, \
             \"crosses_capture_disc\": {}, \"encloses_origin\": {}, \
             \"nearest\": {{\"point_km\": {}, \"width_km\": {:.6}}}, \
             \"farthest\": {{\"point_km\": {}, \"width_km\": {:.6}}}, \"grazing\": {}}}{}",
            c.resonance.h,
            c.resonance.k,
            c.a_prime / AU_M,
            c.center_zeta * km,
            c.radius * km,
            lo * km,
            hi * km,
            c.crosses_capture_disc(frame.capture_radius),
            c.encloses_origin(),
            json_vec2(near_pt, km),
            near.width * km,
            json_vec2(c.farthest_point(), km),
            far.width * km,
            graze_json,
            if i + 1 < circles.len() { "," } else { "" }
        );
    }
    j.push_str("  ]\n}\n");

    if let Some(dir) = std::path::Path::new(&out_path).parent() {
        if !dir.as_os_str().is_empty() {
            std::fs::create_dir_all(dir).expect("create output dir");
        }
    }
    std::fs::write(&out_path, j).expect("write JSON");
    println!("\nwrote {out_path}");
}
