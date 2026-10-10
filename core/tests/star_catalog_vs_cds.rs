//! The star-catalogue gate: the packed Tycho-2 (`tools/fetch_tycho2.py` →
//! `core/src/star_catalog.rs`) against two outside services that share no code
//! with either — VizieR's own cone searches on Tycho-2, and SIMBAD's positions
//! of named stars (`fixtures/star_oracle.txt`, from
//! `pyref/fetch_star_oracle.py`).

use std::collections::BTreeSet;

use asteroid_core::astrometry::{separation_rad, unit_of, RAD_TO_ARCSEC};
use asteroid_core::star_catalog::{self, StarCatalog};

const ORACLE: &str = include_str!("fixtures/star_oracle.txt");

fn open() -> Option<StarCatalog> {
    let path = star_catalog::resolve_for_test("star catalogue vs CDS")?;
    Some(StarCatalog::open(path).expect("open packed catalogue"))
}

/// The same stars as VizieR in an ordinary field, across RA 0h, and over the
/// pole. A star may be missed or added only if it sits within 1″ of the cone's
/// edge — VizieR forms J2000 positions for the supplement's 1991.25 entries
/// itself, and the two need not agree to the last milliarcsecond.
#[test]
fn cones_find_the_stars_vizier_finds() {
    let Some(cat) = open() else { return };
    let mut cones = 0;
    for line in ORACLE.lines().filter(|l| l.starts_with("cone ")) {
        let f: Vec<&str> = line.split_whitespace().collect();
        let (label, ra, dec, radius): (&str, f64, f64, f64) = (
            f[1],
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            f[4].parse().unwrap(),
        );
        let vizier: BTreeSet<String> = f[5..].iter().map(|s| s.to_string()).collect();
        let center = unit_of(ra.to_radians(), dec.to_radians());
        let radius_rad = radius.to_radians();

        // Ask a little wider, so a star just outside can be told apart from one
        // that is simply missing.
        let wide = cat
            .cone(&center, radius_rad + 2.0 / RAD_TO_ARCSEC, 99.0, 0.0)
            .unwrap();
        let edge_arcsec = |s: &star_catalog::Star| {
            (separation_rad(&s.unit_icrf, &center) - radius_rad) * RAD_TO_ARCSEC
        };
        let ours: BTreeSet<String> = wide
            .iter()
            .filter(|s| edge_arcsec(s) <= 0.0)
            .map(|s| format!("{}-{}-{}", s.id.0, s.id.1, s.id.2))
            .collect();

        let mut bad = Vec::new();
        for id in vizier.difference(&ours) {
            let near_edge = wide
                .iter()
                .find(|s| format!("{}-{}-{}", s.id.0, s.id.1, s.id.2) == *id)
                .map(|s| edge_arcsec(s).abs() < 1.0)
                .unwrap_or(false);
            if !near_edge {
                bad.push(format!("missing TYC {id}"));
            }
        }
        for id in ours.difference(&vizier) {
            let s = wide
                .iter()
                .find(|s| format!("{}-{}-{}", s.id.0, s.id.1, s.id.2) == *id)
                .unwrap();
            if edge_arcsec(s).abs() >= 1.0 {
                bad.push(format!(
                    "extra TYC {id} ({:.1}″ inside the edge)",
                    -edge_arcsec(s)
                ));
            }
        }
        println!(
            "  {label:<9} VizieR {:>3}  ours {:>3}  common {:>3}",
            vizier.len(),
            ours.len(),
            vizier.intersection(&ours).count()
        );
        assert!(bad.is_empty(), "{label}: {}", bad.join(", "));
        cones += 1;
    }
    assert_eq!(cones, 3);
}

/// Each named star is where SIMBAD puts it, and carries its name — **or is
/// flagged as having no proper motion**, and then must not be used to measure
/// against. Polaris is the case that taught this (2026-10-10): Tycho-2 has no
/// mean position or proper motion for it (flag `X`, as for 109 445 of its
/// stars, 4.3 %), only its observed position at epoch ~1991.7, so it sits 379
/// mas from SIMBAD's J2000 position — its own motion over those 8.3 years.
///
/// For the rest: Tycho-2's
/// positions (Hipparcos 1997 frame and proper motions) and SIMBAD's (mostly the
/// 2007 Hipparcos re-reduction) differ by the catalogues' own errors — a few to
/// a few tens of milliarcseconds for bright stars, more for a big fuzzy giant
/// like Betelgeuse — so the bound is 0.1″, still 3–5× tighter than anything the
/// screen measures.
#[test]
fn named_stars_sit_where_simbad_puts_them() {
    let Some(cat) = open() else { return };
    assert!(
        cat.named_count() > 3000,
        "names.txt not loaded: {}",
        cat.named_count()
    );
    let mut checked = 0;
    for line in ORACLE.lines().filter(|l| l.starts_with("star ")) {
        let f: Vec<&str> = line.split_whitespace().collect();
        let (name, ra, dec): (&str, f64, f64) =
            (f[1], f[2].parse().unwrap(), f[3].parse().unwrap());
        let simbad = unit_of(ra.to_radians(), dec.to_radians());
        let near = cat.cone(&simbad, 30.0 / RAD_TO_ARCSEC, 99.0, 0.0).unwrap();
        let star = near
            .iter()
            .find(|s| cat.name(s.id).map(|n| n.label()) == Some(name.to_string()))
            .unwrap_or_else(|| panic!("{name}: no star with that name within 30″ of SIMBAD"));
        let miss_mas = separation_rad(&star.unit_icrf, &simbad) * RAD_TO_ARCSEC * 1e3;
        println!(
            "  {name:<11} {}  V {:5.2}  {:7.1} mas from SIMBAD{}",
            star.id,
            star.v_mag,
            miss_mas,
            if star.no_proper_motion {
                "  (no proper motion: not a reference)"
            } else {
                ""
            }
        );
        if name == "Polaris" {
            assert!(
                star.no_proper_motion,
                "Polaris must be flagged: Tycho-2 has no motion for it"
            );
        }
        assert!(
            miss_mas < 100.0 || star.no_proper_motion,
            "{name}: {miss_mas} mas from SIMBAD and not flagged"
        );
        checked += 1;
    }
    assert_eq!(checked, 4);
}
