---
name: keyholes
description: Tier-3 keyholes DONE 2026-09-02 — core/src/keyhole.rs pins the Öpik ξζ frame + b-vector sign, resonant circles in closed form, the 3:4 keyhole FLOWN to an impact; what was corrected, what is unrun, what is next
metadata:
  node_type: memory
  type: project
---

**Keyholes closed 2026-09-02** (`core/src/keyhole.rs`, 12 kernel-free + 3
kernel-gated tests). The spec's last open physics item — the ξ,ζ convention and
the b-vector sign — is **pinned by derivation and measurement**, not adopted:

- `B` points from Earth's centre to the incoming asymptote (hyperbola centre
  `C = a·e·P̂` lies on the asymptote → closest point `b·(Ŝ×ĥ)` = geometry.rs's B);
  gravity bends toward `−B̂`; `probe_keyhole_rotation` measured it 489×.
  `geometry.rs` now has `b_vector_points_at_the_incoming_asymptote`.
- `OpikFrame`: `η̂ = Ŝ`, `ζ̂` anti-parallel to V⊕'s b-plane projection, `ξ̂ = η̂×ζ̂`
  (right-handed ξηζ). **+ζ always raises a′** (bending toward Earth's motion).
- Identity proved & tested: rotation `Ŝ_out = cosδ Ŝ − sinδ B̂` ≡ Valsecchi
  `cos θ′ = [(b²−c²)cosθ + 2cζ sinθ]/(b²+c²)`. Level sets = circles centred on ζ:
  `ζ_c = c sinθ/(cosθ′−cosθ)`, `R = c|sinθ′|/|cosθ′−cosθ|`. Analytic gradient.
  Keyhole width = Δa′_tol/|∇a′| (one capture diameter of return timing).
- **Correction to the recorded reach-probe result:** the 3:4 locus is b =
  3 866..153 540 km, i.e. it passes THROUGH the capture disc (as do all 27 h≤7
  circles), not 60 843..153 511 — the probe bisected outward from b_cap and could
  not see rays entering the circle inside the disc. The far end was right.
  Nearest *miss* on 3:4 = the grazing point; width 0.18 km there, 24.92 km far.
- `probe_keyhole_map` → `docs/keyhole_map.json` + `tools/keyhole_map_{svg,html}.py`
  → `docs/keyhole_map.{svg,html}`. Two flown deflections (±0.2 m/s @ epoch0)
  match the closed-form a′ to 1.26e-4 / 2.14e-5. Ellipse major axis 89.7° from ξ
  (along ζ = timing). Nominal at ξ 6 695, ζ −2 301 km.
- **THE 3:4 KEYHOLE FLOWN** (`probe_keyhole_return`, 252 s): closed-form aim
  0.216438 m/s retro @ epoch0 → return 53 841 km (~5 cap radii, not the doc's
  ~100); golden-section on Δv → **0.216550 m/s → 1 130 km from Earth's centre on
  2042-12-31**, 3.00 yr after the 2040-01-01 flyby = an impact keyhole. Δv window
  ~1.3e-5 m/s ≈ 13 km of b ≈ the closed-form width. Pinned by
  `the_three_four_keyhole_returns_the_rock_to_earth_when_flown` (26 s, re-flies
  that Δv, asserts return < 4 cap radii at 2.9–3.1 yr).
- Godot: binding's `encounter_basis()` is now the pinned Öpik frame (fallback to
  ecliptic-pole basis + `bplane_frame_pinned()` false); `keyhole_circles(max_years)`
  #[func] → Array of Dictionary; `encounter.gd` draws circles + labels, `[H]`
  toggles (`encounter_keyholes`, keycode 72, in project.godot); readout FRAME line
  says OPIK. **GDScript UNRUN** (no Godot in the session); binding tests pass
  (`the_bplane_axes_are_the_pinned_opik_frame_and_the_keyhole_map_is_the_cores`).
- Infra: `.github/workflows/ci.yml` (fmt/clippy/kernel-free, then kernels
  cached + REQUIRE_KERNELS core suite), `tools/fetch_kernels.py` (NAIF →
  **naif-de440 PyPI wheel** fallback found because the session proxy blocked NAIF
  and nyx-space; pck11.pca via GitHub LFS media URL), `DEVELOPING.md`, README
  rewritten (status was stale: "Early"). One fmt-only commit (26 files).
- Not done here: Horizons `.neo` tests could not run (JPL API blocked — 3 tests
  fail under REQUIRE_KERNELS on this box only); the Tier-3 ellipse is not on the
  Godot view; keyhole *targeting* is a probe loop, not a core API yet.
