//! Finding the JPL DE kernels on this machine — the Rust mirror of
//! `godot/scripts/kernels.gd`.
//!
//! # Why this exists
//!
//! Every kernel-gated test in this workspace used to read `ASTEROID_DE_KERNEL`
//! and `ASTEROID_PLANETARY_CONSTANTS` directly and `return` when either was
//! unset. That is the right *shape* — a machine genuinely without the kernels
//! (they are hundreds of megabytes and are not in git) must still get a green
//! suite. But it hid a trap that bit for real on 2026-07-17:
//!
//! > `cargo test` in a shell that had not exported the two variables silently
//! > skipped roughly half the physics suite and printed **"13 passed"**. The
//! > `eprintln!` in each skip branch is swallowed — cargo releases a passing
//! > test's stderr only for *failing* tests. The machine *had* the kernels;
//! > only the environment was unset. Two verification claims in that batch were
//! > vacuous as a result: a rescale "confirmed" by a test that never ran, and a
//! > new equivalence test that had never once executed.
//!
//! The only tell was the wall clock — 0.02 s versus 69 s — which is not a thing
//! a reader of a green CI log ever sees. The GDScript side never had this
//! problem because `Kernels.resolve()` falls back from the environment to a
//! config file to a list of conventional directories, so the *game* runs real
//! physics either way. This module gives the Rust harness the same resolver,
//! and then goes one step further (see [`resolve_for_test`]).
//!
//! # Two separate failures, two separate fixes
//!
//! Resolution alone cures **"I have the kernels but did not point at them"** —
//! today, on this box. It does nothing about **"everything skipped and printed
//! green"**, which is the failure that actually cost the work: a fresh clone, a
//! CI container, or a renamed directory puts it right back. So:
//!
//! - [`resolve`] finds the pair. Absent → `None` → the caller skips, green,
//!   exactly as before. Offline CI is preserved on purpose.
//! - [`ASTEROID_REQUIRE_KERNELS`] turns that `None` into a **panic**. Set it in
//!   any run whose green is supposed to *mean* something, and a skipped suite
//!   becomes a loud failure instead of a quiet lie.
//!
//! ```sh
//! ASTEROID_REQUIRE_KERNELS=1 cargo test --release   # green here means it ran
//! cargo test --release                              # green may mean "skipped"
//! ```
//!
//! # The third kernel
//!
//! [`KernelPair`] also carries an **optional** small-body kernel
//! ([`sb441-n16.bsp`](KernelPair::small_bodies)) when one is present. It is
//! deliberately outside the both-or-nothing rule: it is 646 MB, a fresh clone
//! will not have it, and everything that worked before it existed works without
//! it. Absent → `None` → the caller does without. It is never a resolution
//! failure, because failing the *pair* over it would take the planets down too.
//!
//! # What is deliberately not here
//!
//! `kernels.gd` reads `user://kernels.cfg` between the environment and the
//! directory scan. This module does not: `user://` resolves through Godot's own
//! per-platform app-data path, and hand-reconstructing that in Rust to read a
//! file written by the frontend would be a guess that silently rots. The
//! directory scan covers the case the config file exists for, and a caller that
//! knows better can always pass explicit paths (`MissionCore::load_from`).

use std::path::{Path, PathBuf};

/// Environment variable naming the DE ephemeris `.bsp` explicitly.
pub const ENV_BSP: &str = "ASTEROID_DE_KERNEL";

/// Environment variable naming the planetary-constants `.pca` explicitly.
pub const ENV_PCA: &str = "ASTEROID_PLANETARY_CONSTANTS";

/// Set this (to anything non-empty) to make an unresolvable kernel pair a hard
/// failure instead of a silent skip. See the module docs — this is the half of
/// the fix that keeps the silent-green failure from coming back.
pub const ENV_REQUIRE: &str = "ASTEROID_REQUIRE_KERNELS";

/// Environment variable naming the small-body `.bsp` explicitly. **Optional** —
/// see [`KernelPair::small_bodies`].
pub const ENV_SMALL_BODIES: &str = "ASTEROID_SMALL_BODY_KERNEL";

/// Accepted DE kernel filenames, most-preferred first. `de440s` is the standard
/// short span (~1850–2149); `de441` is the long span. The core copes with
/// either, so this is a preference order, not a requirement.
const BSP_NAMES: &[&str] = &["de440s.bsp", "de440.bsp", "de441.bsp"];

/// Accepted planetary-constants filenames. `pck11.pca` is what the tests pin.
const PCA_NAMES: &[&str] = &["pck11.pca"];

/// Accepted small-body ephemeris filenames. `sb441-n16.bsp` is the 16 asteroid
/// perturbers ASSIST uses — Ceres through Interamnia, main-belt, Sun-centered.
const SMALL_BODY_NAMES: &[&str] = &["sb441-n16.bsp"];

/// A resolved kernel pair, and where it came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KernelPair {
    /// Path to the DE ephemeris `.bsp`.
    pub bsp: PathBuf,
    /// Path to the planetary-constants `.pca`.
    pub pca: PathBuf,
    /// Path to a small-body ephemeris `.bsp`, if one is present — **optional by
    /// construction, and it must stay that way.**
    ///
    /// `sb441-n16.bsp` is ~646 MB, twenty times the DE kernel, and a fresh clone
    /// will not have it. Everything the project did before it existed still works
    /// without it, so a missing small-body kernel is `None` and never a resolution
    /// failure: the pair is still a pair. It is a *third, independent* thing, not
    /// a third member of the both-or-nothing rule that governs `bsp` + `pca`.
    ///
    /// Mounting it is not free — measured **5.7 s cold**, 272 ms warm, which is
    /// page-cache I/O on the 646 MB file. That is why nothing on a startup path
    /// mounts it; see the build worker in `godot/rust`.
    pub small_bodies: Option<PathBuf>,
    /// Human-readable origin of the hit — the environment, or the directory
    /// scanned. When resolution goes wrong, *which* mechanism answered is the
    /// first thing worth knowing.
    pub source: String,
}

impl KernelPair {
    /// The pair as the `(&str, &str)` most call sites want to hand to
    /// [`Ephemeris::load`](crate::ephemeris::Ephemeris::load) and
    /// [`with_constants`](crate::ephemeris::Ephemeris::with_constants).
    ///
    /// # Panics
    /// If either path is not valid UTF-8, which no path this resolver produces
    /// can be — they come from UTF-8 env vars and ASCII filenames.
    pub fn as_strs(&self) -> (&str, &str) {
        (
            self.bsp.to_str().expect("kernel path is UTF-8"),
            self.pca.to_str().expect("kernel path is UTF-8"),
        )
    }

    /// The small-body kernel as `&str`, or `None`. Separate from
    /// [`as_strs`](Self::as_strs) because it is separately optional — a caller
    /// that wants the pair should not have to think about this one at all.
    ///
    /// # Panics
    /// If the path is not valid UTF-8; see [`as_strs`](Self::as_strs).
    pub fn small_bodies_str(&self) -> Option<&str> {
        self.small_bodies
            .as_ref()
            .map(|p| p.to_str().expect("kernel path is UTF-8"))
    }
}

/// Resolve the kernel pair, first hit wins:
///
/// 1. `ASTEROID_DE_KERNEL` + `ASTEROID_PLANETARY_CONSTANTS` (explicit paths)
/// 2. the conventional directories from [`search_dirs`]
///
/// **Both or nothing.** Half a pair is a misconfiguration worth falling through
/// rather than silently pairing an env kernel with a scanned one — the same
/// rule `kernels.gd::_from_env` follows.
pub fn resolve() -> Option<KernelPair> {
    if let (Ok(bsp), Ok(pca)) = (std::env::var(ENV_BSP), std::env::var(ENV_PCA)) {
        let (bsp, pca) = (PathBuf::from(bsp), PathBuf::from(pca));
        if bsp.is_file() && pca.is_file() {
            // Beside the DE kernel is where the scan would have looked, so an
            // env-resolved pair gets the same small-body courtesy.
            let small_bodies = bsp.parent().and_then(small_bodies_in);
            return Some(KernelPair {
                bsp,
                pca,
                small_bodies,
                source: format!("{ENV_BSP} env"),
            });
        }
    }

    for dir in search_dirs() {
        if let Some(pair) = scan_dir(&dir) {
            return Some(pair);
        }
    }

    None
}

/// [`resolve`], with the skip made auditable — the entry point every
/// kernel-gated test should use.
///
/// - Resolved → `Some(pair)`, and the test runs.
/// - Unresolved, [`ENV_REQUIRE`] unset → `None`, and the caller returns green.
///   A machine that genuinely lacks the kernels still passes.
/// - Unresolved, [`ENV_REQUIRE`] set → **panic**, naming `what` and every place
///   searched. This is what makes a skipped suite visible instead of green.
///
/// ```ignore
/// let Some(k) = kernels::resolve_for_test("geocenter reconstruction") else {
///     return;
/// };
/// ```
#[must_use]
pub fn resolve_for_test(what: &str) -> Option<KernelPair> {
    match resolve() {
        Some(pair) => Some(pair),
        None if require_kernels() => panic!(
            "{ENV_REQUIRE} is set but no DE kernel pair resolved, so \"{what}\" \
             would have skipped and printed green.\n{}",
            not_found_message()
        ),
        None => {
            // Swallowed by cargo for a passing test — which is exactly why the
            // panic branch above exists. Kept for `--nocapture` runs.
            eprintln!("no DE kernels resolved — skipping {what}");
            None
        }
    }
}

/// Whether [`ENV_REQUIRE`] is set to something non-empty.
pub fn require_kernels() -> bool {
    std::env::var(ENV_REQUIRE).is_ok_and(|v| !v.is_empty())
}

/// Conventional kernel directories, in scan order. Mirrors
/// `kernels.gd::_search_dirs`, anchored on this crate's source location rather
/// than the process working directory so it answers the same from `cargo test`
/// at the workspace root, from a crate subdirectory, or from an example.
pub fn search_dirs() -> Vec<PathBuf> {
    // `<repo>/core` at build time. The kernels are ordinary large files beside
    // the repo, never inside it (they are gitignored and far too big to commit).
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let repo = crate_root.parent().unwrap_or(crate_root);
    let mut dirs = vec![
        // A kernels/ folder in the repo — the natural "drop them here" spot for
        // a fresh clone, and the first thing a new contributor will try.
        repo.join("kernels"),
        // This project's conventional scratch root (../temp/AsteroidDefense),
        // which is where the dev machine's kernels actually live.
        repo.join("../temp/AsteroidDefense/kernels"),
    ];
    // Beside the current executable — where a shipped build would carry them.
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            dirs.push(dir.join("kernels"));
        }
    }
    dirs
}

/// The first `[bsp, pca]` pair present in `dir`, or `None` if it lacks either.
fn scan_dir(dir: &Path) -> Option<KernelPair> {
    let bsp = first_present(dir, BSP_NAMES)?;
    let pca = first_present(dir, PCA_NAMES)?;
    Some(KernelPair {
        bsp,
        pca,
        small_bodies: small_bodies_in(dir),
        source: dir.display().to_string(),
    })
}

/// The small-body kernel: [`ENV_SMALL_BODIES`] if it names a real file,
/// otherwise whatever sits in `dir`. Unlike the pair this never fails — it
/// answers `None` and the caller does without.
fn small_bodies_in(dir: &Path) -> Option<PathBuf> {
    if let Ok(p) = std::env::var(ENV_SMALL_BODIES) {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    first_present(dir, SMALL_BODY_NAMES)
}

fn first_present(dir: &Path, names: &[&str]) -> Option<PathBuf> {
    names
        .iter()
        .map(|n| dir.join(n))
        .find(|p| p.is_file())
}

/// Every place looked and both ways to fix it. A bare "kernels not found" would
/// send someone hunting through source for the search order.
pub fn not_found_message() -> String {
    let mut lines = vec![
        format!(
            "no DE kernel pair found (need one of [{}] + [{}])",
            BSP_NAMES.join(", "),
            PCA_NAMES.join(", ")
        ),
        format!("searched: {ENV_BSP} + {ENV_PCA} env"),
    ];
    for d in search_dirs() {
        lines.push(format!("searched: {}", d.display()));
    }
    lines.push(format!(
        "fix: export {ENV_BSP}=<...>/de440s.bsp and {ENV_PCA}=<...>/pck11.pca, \
         or put both in one of the directories above"
    ));
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The search list is anchored on the crate source, not the working
    /// directory — so it cannot change answer depending on where cargo was
    /// invoked from. (The 0.02 s-vs-69 s trap was hard enough to see without a
    /// resolver that also moves underfoot.)
    #[test]
    fn search_dirs_are_absolute_and_stable() {
        let dirs = search_dirs();
        assert!(!dirs.is_empty(), "no conventional directories to scan");
        // The two repo-anchored entries are built from CARGO_MANIFEST_DIR, which
        // is absolute; the exe-relative one comes from current_exe(), also
        // absolute. A relative entry here would mean a working-directory
        // dependency sneaked in.
        for d in &dirs {
            assert!(d.is_absolute(), "{} is not absolute", d.display());
        }
        assert_eq!(dirs, search_dirs(), "search order is not deterministic");
    }

    /// Half a pair resolves to nothing rather than to a mismatched pairing.
    /// Scanning a directory that holds neither file must simply decline.
    #[test]
    fn scan_dir_rejects_incomplete_directories() {
        assert!(scan_dir(Path::new("does/not/exist")).is_none());
        // The repo root has no kernels in it (they are gitignored and live
        // outside), so it stands in for "a directory with neither file".
        let repo = Path::new(env!("CARGO_MANIFEST_DIR"));
        assert!(scan_dir(&repo.join("src")).is_none());
    }

    /// A directory with the pair but no small-body kernel still resolves, with
    /// `small_bodies: None`. This is the whole optionality contract: `sb441` is
    /// 646 MB and a fresh clone has none, so if its absence could ever fail a
    /// resolution, every machine without it would lose the planets too.
    #[test]
    fn small_body_kernel_is_optional() {
        // The env override outranks the directory by design, so it would mask
        // both assertions below. Nothing to check here on a shell that sets it.
        if std::env::var(ENV_SMALL_BODIES).is_ok() {
            return;
        }
        let dir = std::env::temp_dir().join(format!(
            "asteroid_kernels_optional_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        // Empty files are enough: the scan tests `is_file`, it does not parse.
        std::fs::write(dir.join("de440s.bsp"), b"").expect("write bsp");
        std::fs::write(dir.join("pck11.pca"), b"").expect("write pca");

        let pair = scan_dir(&dir).expect("pair without a small-body kernel must resolve");
        assert_eq!(pair.small_bodies, None, "invented a small-body kernel");

        // …and it is found when present, so `None` above means absent, not blind.
        std::fs::write(dir.join("sb441-n16.bsp"), b"").expect("write sb");
        let pair = scan_dir(&dir).expect("still a pair");
        assert_eq!(pair.small_bodies, Some(dir.join("sb441-n16.bsp")));

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The not-found text names every searched location — the message is the
    /// repair path, so an incomplete one is a real defect.
    #[test]
    fn not_found_message_lists_every_search_location() {
        let msg = not_found_message();
        assert!(msg.contains(ENV_BSP), "message omits the env var to set");
        for d in search_dirs() {
            assert!(
                msg.contains(&d.display().to_string()),
                "message omits searched dir {}",
                d.display()
            );
        }
    }
}
