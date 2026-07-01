//! `StateVector` — an instantaneous position + velocity in the core's frame.
//!
//! The state is expressed in the **barycentric (SSB) ICRF** frame in **SI
//! units** — position in metres, velocity in metres/second (HANDOFF §5). This is
//! the phase-space point the integrator advances and the Cartesian side of the
//! element↔state conversion in [`crate::elements`].
//!
//! Note on frames: a `StateVector` carries no frame tag of its own. The core's
//! convention is that every state is barycentric-ICRF/SI; the *relative* state
//! passed to the Keplerian conversions ([`crate::elements::OrbitalElements`]) is
//! understood to be relative to the attracting body whose `μ` is supplied.

use nalgebra::Vector3;

/// Position and velocity at a single epoch, barycentric ICRF, SI units.
///
/// `position` is in metres, `velocity` in metres per second. Copy-cheap (48
/// bytes); pass by value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateVector {
    /// Position vector, metres, ICRF.
    pub position: Vector3<f64>,
    /// Velocity vector, metres per second, ICRF.
    pub velocity: Vector3<f64>,
}

impl StateVector {
    /// Construct from position (m) and velocity (m/s) vectors.
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>) -> Self {
        Self { position, velocity }
    }

    /// Construct from raw components: position `(rx, ry, rz)` in metres,
    /// velocity `(vx, vy, vz)` in metres/second.
    #[allow(clippy::too_many_arguments)]
    pub fn from_components(rx: f64, ry: f64, rz: f64, vx: f64, vy: f64, vz: f64) -> Self {
        Self {
            position: Vector3::new(rx, ry, rz),
            velocity: Vector3::new(vx, vy, vz),
        }
    }

    /// Distance from the frame origin, metres (`‖position‖`).
    pub fn radius(&self) -> f64 {
        self.position.norm()
    }

    /// Speed, metres/second (`‖velocity‖`).
    pub fn speed(&self) -> f64 {
        self.velocity.norm()
    }
}
