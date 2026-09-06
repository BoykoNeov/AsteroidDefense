//! The one obliquity rotation: ecliptic-J2000 ↔ ICRF (equatorial-J2000).
//!
//! Every frame in this crate is ICRF — the integrator, the `.neo` tables, the
//! b-plane geometry. Two things arrive in the *ecliptic* frame instead and have
//! to be rotated on the way in: a designer orbit authored with its inclination
//! referred to the plane a human thinks in, and JPL's published osculating
//! elements for a real small body ([`crate::sbdb`]), which the Small-Body
//! Database gives in ecliptic-J2000 by convention. The display side runs the
//! rotation the other way, to draw a flat solar system rather than one tilted by
//! 23.4°.
//!
//! There is exactly one obliquity constant in the project and it lives here.
//! The alternative — the constant written out again wherever a rotation is
//! needed — is the same failure this codebase already refuses for `μ_sun`: two
//! spellings of one physical constant that agree until one of them is edited,
//! and a silent bias afterwards. The Godot binding's `icrf_km_to_ecliptic_au`
//! and its ecliptic-pole helper delegate here.
//!
//! # Which obliquity
//!
//! `84381.448″`, the IAU 1976 mean obliquity at J2000 — **not** the IAU 2006
//! value (`84381.406″`). This is not a precision question but a compatibility
//! one: `84381.448″` is the number that *defines* SPICE's `ECLIPJ2000` frame, so
//! using it makes our ecliptic the same plane the kernels and Horizons mean by
//! the name. The 42 milliarcsecond difference would otherwise show up as a
//! ~30 km cross-track offset at 1 au, comparable to quantities this project
//! measures.

use nalgebra::Vector3;

/// Mean obliquity of the ecliptic at J2000, arcseconds — the exact value that
/// defines SPICE's `ECLIPJ2000` frame.
pub const OBLIQUITY_ARCSEC: f64 = 84_381.448;

/// The same angle in radians.
pub fn obliquity_rad() -> f64 {
    OBLIQUITY_ARCSEC / 3600.0 * std::f64::consts::PI / 180.0
}

/// Rotate an **ecliptic-J2000** vector into ICRF (equatorial-J2000).
///
/// A rotation by `+ε` about the shared X axis (the vernal equinox), and
/// **unit-agnostic** — it rotates a vector, so the same call maps a position and
/// a velocity. Note that `x` is untouched by construction; that is the cheapest
/// available sanity check on a suspected frame mix-up.
pub fn ecliptic_to_icrf(v: Vector3<f64>) -> Vector3<f64> {
    let (s, c) = obliquity_rad().sin_cos();
    Vector3::new(v.x, c * v.y - s * v.z, s * v.y + c * v.z)
}

/// Rotate an **ICRF** (equatorial-J2000) vector into ecliptic-J2000 — the exact
/// inverse of [`ecliptic_to_icrf`], a rotation by `−ε` about X.
pub fn icrf_to_ecliptic(v: Vector3<f64>) -> Vector3<f64> {
    let (s, c) = obliquity_rad().sin_cos();
    Vector3::new(v.x, c * v.y + s * v.z, -s * v.y + c * v.z)
}

/// The ecliptic north pole **expressed in ICRF** — `(0, −sin ε, cos ε)`.
///
/// Not `(0, 0, 1)`: that is the pole in *ecliptic* coordinates, and anything
/// dotting it against an ICRF vector is off by the 23.4° obliquity.
pub fn ecliptic_north_icrf() -> Vector3<f64> {
    let (s, c) = obliquity_rad().sin_cos();
    Vector3::new(0.0, -s, c)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two directions must compose to the identity, or one of them has the
    /// sign of `ε` wrong — the single most likely mistake in this file, and one
    /// that a one-way test cannot see.
    #[test]
    fn the_two_rotations_are_inverses() {
        for v in [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(-6.13e10, 1.439e11, -9.12e9),
        ] {
            let back = icrf_to_ecliptic(ecliptic_to_icrf(v));
            assert!(
                (back - v).norm() <= 1e-9 * v.norm().max(1.0),
                "round trip moved {v:?} to {back:?}"
            );
        }
    }

    /// A rotation about X leaves X alone and preserves length. Both directions,
    /// because a transposed matrix would still preserve length.
    #[test]
    fn it_is_a_rotation_about_the_x_axis() {
        let v = Vector3::new(-6.13e10, 1.439e11, -9.12e9);
        for w in [ecliptic_to_icrf(v), icrf_to_ecliptic(v)] {
            assert_eq!(w.x, v.x, "x must be untouched by a rotation about x");
            assert!((w.norm() - v.norm()).abs() <= 1e-6, "length not preserved");
        }
    }

    /// The ecliptic pole in ICRF is what `ecliptic_to_icrf` maps ecliptic `ẑ`
    /// onto — stated twice in the module, so pin that the two agree.
    #[test]
    fn the_pole_helper_is_the_rotation_of_ecliptic_z() {
        let mapped = ecliptic_to_icrf(Vector3::new(0.0, 0.0, 1.0));
        let pole = ecliptic_north_icrf();
        assert!(
            (mapped - pole).norm() < 1e-15,
            "pole {pole:?} disagrees with rotated ẑ {mapped:?}"
        );
    }

    /// The obliquity is `ECLIPJ2000`'s, not IAU 2006's. Pinned because the two
    /// differ by 42 mas — invisible in any round-trip test and worth ~30 km at
    /// 1 au against a kernel that uses the other one.
    #[test]
    fn the_obliquity_is_the_spice_eclipj2000_value() {
        assert_eq!(OBLIQUITY_ARCSEC, 84_381.448);
        assert!((obliquity_rad() - 0.409_092_804_222_328_97).abs() < 1e-15);
    }
}
