//! Ephemeris loading — the entry point for perturber positions.
//!
//! The Tier-1 MVP integrates the asteroid as a *test particle* in the DE440/441
//! ephemeris field, so the core needs perturber **positions** (not just GM
//! constants) from day one. This module is where a local SPICE/DE kernel is
//! loaded via ANISE.
//!
//! **Scaffold status (HANDOFF §10 task 1):** this is an intentional stub. The
//! task-0.5 de-risk spike (§10 task 2) is what proves the real ANISE
//! DE-position reader returns a sane reconstructed **geocenter** (not the EMB)
//! for a known epoch. Do not encode an assumed ANISE API shape here before that
//! spike runs — kernels are loaded from a local path only (offline, no
//! network auto-download), so the loader takes an explicit path.

use std::path::{Path, PathBuf};

/// Errors that can arise while loading an ephemeris kernel.
#[derive(Debug)]
pub enum EphemerisError {
    /// The requested kernel path does not exist on disk.
    NotFound(PathBuf),
    /// The loader is not implemented yet (filled in by the task-0.5 spike).
    NotImplemented,
}

impl std::fmt::Display for EphemerisError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EphemerisError::NotFound(p) => write!(f, "ephemeris kernel not found: {}", p.display()),
            EphemerisError::NotImplemented => {
                write!(f, "ephemeris loader not implemented (see HANDOFF §10 task 0.5)")
            }
        }
    }
}

impl std::error::Error for EphemerisError {}

/// Handle to a loaded ephemeris. Wraps the ANISE almanac once the task-0.5
/// spike wires it in; for now it only remembers which kernel it was asked for.
#[derive(Debug)]
pub struct Ephemeris {
    kernel_path: PathBuf,
}

impl Ephemeris {
    /// Load a DE kernel from a local path. Validates the path exists so callers
    /// fail loudly and offline; the actual ANISE almanac load + geocenter
    /// reconstruction is wired in by the task-0.5 de-risk spike.
    pub fn load(kernel_path: impl AsRef<Path>) -> Result<Self, EphemerisError> {
        let kernel_path = kernel_path.as_ref().to_path_buf();
        if !kernel_path.exists() {
            return Err(EphemerisError::NotFound(kernel_path));
        }
        // TODO(task-0.5): construct the ANISE almanac from `kernel_path`,
        // confirm it reconstructs a geocenter (not the EMB) for a known epoch.
        Err(EphemerisError::NotImplemented)
    }

    /// Path of the kernel this handle was created for.
    pub fn kernel_path(&self) -> &Path {
        &self.kernel_path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_kernel_reports_not_found() {
        let err = Ephemeris::load("does/not/exist/de440.bsp").unwrap_err();
        assert!(matches!(err, EphemerisError::NotFound(_)));
    }
}
