//! Real background stars — the Tycho-2 catalogue, packed by
//! `tools/fetch_tycho2.py` into `kernels/stars/`.
//!
//! The sky-observation screen shows the asteroid moving against these
//! (`docs/plans/2026-10-10-sky-observation-screen.md`), and the player measures
//! the rock's position *against* them, as a real astronomer does. So they must
//! sit in the same frame as [`astrometry`](crate::astrometry)'s rock: Tycho-2
//! positions are ICRS at epoch J2000 with proper motions, and a star is carried
//! to the shot's date by its proper motion and nothing else (no aberration — see
//! the astrometry module doc for why).
//!
//! # Reading only what a field needs
//!
//! The packed file is ~60 MB, sorted into 1°×1° cells of J2000 position with an
//! offset table in front. A query reads the offset table once (at open) and then
//! only the rows of cells its cone touches — a telescope field is a handful of
//! cells, a few hundred kilobytes. Reading the whole file up front would be the
//! same mistake as the 11 s DE440 load on a cold spinning disk (HANDOFF,
//! *Frontend startup*).
//!
//! # Absent catalogue
//!
//! Like the kernels it is gitignored and fetched. [`resolve`] answers `None`
//! when it is missing; [`resolve_for_test`] panics under
//! `ASTEROID_REQUIRE_KERNELS=1` so a suite that skipped the star checks cannot
//! print green.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use nalgebra::Vector3;

use crate::astrometry::{radec_of, unit_of};

/// Environment variable naming the packed catalogue file explicitly.
pub const ENV_STARS: &str = "ASTEROID_STARS";

const MAGIC: &[u8; 8] = b"ASTSTARS";
const VERSION: u32 = 1;
const CELLS: usize = 360 * 180;
const RECORD_BYTES: usize = 24;
const HEADER_BYTES: u64 = 8 + 4 + 4 + 4 + 4 * (CELLS as u64 + 1);
const MAS_TO_RAD: f64 = std::f64::consts::PI / (180.0 * 3_600_000.0);
const JULIAN_YEAR_S: f64 = 365.25 * 86_400.0;

/// The Tycho identifier, `TYC tyc1-tyc2-tyc3`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TycId(pub u16, pub u16, pub u8);

impl std::fmt::Display for TycId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TYC {}-{}-{}", self.0, self.1, self.2)
    }
}

/// One star, carried to the date it was asked for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Star {
    pub id: TycId,
    /// ICRF direction at the requested date.
    pub unit_icrf: Vector3<f64>,
    pub ra_rad: f64,
    pub dec_rad: f64,
    /// Johnson V, from Tycho BT/VT (see `tools/fetch_tycho2.py::v_mag`).
    pub v_mag: f32,
    /// From Tycho-2's supplement 1 (mostly the brightest stars).
    pub from_supplement: bool,
    /// The catalogue gives no proper motion, so the position is the *observed*
    /// one at epoch ~1991 (Tycho-2 flag `X`, 109 445 stars, and supplement-1's
    /// Tycho-1 entries) and goes stale at the star's unknown motion — typically
    /// 0.2–0.8″ by the 2020s, 0.38″ for Polaris. Drawn, but **never used as a
    /// reference star** to measure the asteroid against.
    pub no_proper_motion: bool,
}

/// What a star is called, where it has a name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StarName {
    /// Bayer letter as the catalogue abbreviates it (`alf`, `bet`, `eta02`), or
    /// empty.
    pub bayer: String,
    /// Flamsteed number as text, or empty.
    pub flamsteed: String,
    /// IAU constellation abbreviation (`Lyr`).
    pub constellation: String,
    /// Proper name (`Vega`), or empty.
    pub proper: String,
}

impl StarName {
    /// The label to draw: the proper name if there is one, else the Bayer
    /// letter (as a Greek letter) with the constellation, else the Flamsteed
    /// number with it.
    pub fn label(&self) -> String {
        if !self.proper.is_empty() {
            return self.proper.clone();
        }
        if !self.bayer.is_empty() {
            return format!("{} {}", greek(&self.bayer), self.constellation);
        }
        if !self.flamsteed.is_empty() {
            return format!("{} {}", self.flamsteed, self.constellation);
        }
        String::new()
    }
}

/// `alf` → `α`, `eta02` → `η²`, `mu.` → `μ`; anything unrecognised is returned
/// unchanged. The spellings are the cross index's own (CDS IV/27A): theta is
/// `the`, and the two-letter names are padded with a dot (`mu.`, `nu.`, `pi.`) —
/// read off the shipped `names.txt`, where a guessed `tet` had left every θ
/// star printed as `the Hya`.
fn greek(abbrev: &str) -> String {
    const LETTERS: [(&str, &str); 24] = [
        ("alf", "α"),
        ("bet", "β"),
        ("gam", "γ"),
        ("del", "δ"),
        ("eps", "ε"),
        ("zet", "ζ"),
        ("eta", "η"),
        ("the", "θ"),
        ("iot", "ι"),
        ("kap", "κ"),
        ("lam", "λ"),
        ("mu", "μ"),
        ("nu", "ν"),
        ("ksi", "ξ"),
        ("omi", "ο"),
        ("pi", "π"),
        ("rho", "ρ"),
        ("sig", "σ"),
        ("tau", "τ"),
        ("ups", "υ"),
        ("phi", "φ"),
        ("chi", "χ"),
        ("psi", "ψ"),
        ("ome", "ω"),
    ];
    let letters: String = abbrev
        .chars()
        .take_while(|c| c.is_ascii_alphabetic())
        .collect();
    // Only the component number after the letters; the padding dot is dropped.
    let digits: String = abbrev
        .chars()
        .skip(letters.len())
        .filter(|c| c.is_ascii_digit())
        .collect();
    let Some(&(_, g)) = LETTERS.iter().find(|(a, _)| *a == letters) else {
        return abbrev.to_string();
    };
    let sup: String = digits
        .trim_start_matches('0')
        .chars()
        .map(|c| match c {
            '1' => '¹',
            '2' => '²',
            '3' => '³',
            '4' => '⁴',
            '5' => '⁵',
            '6' => '⁶',
            '7' => '⁷',
            '8' => '⁸',
            '9' => '⁹',
            _ => '⁰',
        })
        .collect();
    format!("{g}{sup}")
}

/// Why the catalogue could not be read.
#[derive(Debug)]
pub enum StarCatalogError {
    Io(std::io::Error),
    /// Not a packed catalogue, or a different format revision.
    Format(String),
}

impl std::fmt::Display for StarCatalogError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "star catalogue: {e}"),
            Self::Format(m) => write!(f, "star catalogue: {m}"),
        }
    }
}

impl std::error::Error for StarCatalogError {}

impl From<std::io::Error> for StarCatalogError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

trait Source: Read + Seek + Send {}
impl<T: Read + Seek + Send> Source for T {}

/// The packed Tycho-2 catalogue, opened for cone queries.
pub struct StarCatalog {
    source: Mutex<Box<dyn Source>>,
    offsets: Vec<u32>,
    len: usize,
    max_pm_mas_yr: f64,
    names: HashMap<TycId, StarName>,
}

impl std::fmt::Debug for StarCatalog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StarCatalog")
            .field("len", &self.len)
            .field("names", &self.names.len())
            .finish()
    }
}

impl StarCatalog {
    /// Open `tycho2.stars`, and `names.txt` beside it if present.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, StarCatalogError> {
        let path = path.as_ref();
        let file = BufReader::new(File::open(path)?);
        let mut cat = Self::from_source(Box::new(file))?;
        let names_path = path.with_file_name("names.txt");
        if let Ok(text) = std::fs::read_to_string(&names_path) {
            cat.names = parse_names(&text)?;
        }
        Ok(cat)
    }

    fn from_source(mut source: Box<dyn Source>) -> Result<Self, StarCatalogError> {
        let mut head = [0u8; 20];
        source.read_exact(&mut head)?;
        if &head[..8] != MAGIC {
            return Err(StarCatalogError::Format(
                "not a packed star catalogue".into(),
            ));
        }
        let version = u32::from_le_bytes(head[8..12].try_into().unwrap());
        if version != VERSION {
            return Err(StarCatalogError::Format(format!(
                "format version {version}, this reader understands {VERSION}"
            )));
        }
        let len = u32::from_le_bytes(head[12..16].try_into().unwrap()) as usize;
        let max_pm = f32::from_le_bytes(head[16..20].try_into().unwrap()) as f64;
        let mut table = vec![0u8; 4 * (CELLS + 1)];
        source.read_exact(&mut table)?;
        let offsets: Vec<u32> = table
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| u32::from_le_bytes(*b))
            .collect();
        if offsets[CELLS] as usize != len || offsets.windows(2).any(|w| w[0] > w[1]) {
            return Err(StarCatalogError::Format(
                "offset table is inconsistent".into(),
            ));
        }
        Ok(Self {
            source: Mutex::new(source),
            offsets,
            len,
            max_pm_mas_yr: max_pm,
            names: HashMap::new(),
        })
    }

    /// Number of stars in the file.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the file holds no stars.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The star's name, if it has one.
    pub fn name(&self, id: TycId) -> Option<&StarName> {
        self.names.get(&id)
    }

    /// Number of named stars loaded.
    pub fn named_count(&self) -> usize {
        self.names.len()
    }

    /// Every star brighter than `mag_limit` within `radius_rad` of `center`
    /// (any length), at `tdb_seconds` past J2000 — positions carried there by
    /// proper motion. Brightest first.
    pub fn cone(
        &self,
        center: &Vector3<f64>,
        radius_rad: f64,
        mag_limit: f32,
        tdb_seconds: f64,
    ) -> Result<Vec<Star>, StarCatalogError> {
        let center = center.normalize();
        let years = tdb_seconds / JULIAN_YEAR_S;
        // Search the J2000 cells with the radius padded by the farthest any star
        // in the file can have moved since J2000.
        let pad = self.max_pm_mas_yr * years.abs() * MAS_TO_RAD;
        let search = radius_rad + pad;
        let cos_radius = radius_rad.cos();

        let (ra_c, dec_c) = radec_of(&center);
        let dec_lo = (dec_c - search).to_degrees().max(-90.0);
        let dec_hi = (dec_c + search).to_degrees().min(90.0);
        let row_lo = ((dec_lo + 90.0).floor() as usize).min(179);
        let row_hi = ((dec_hi + 90.0).floor() as usize).min(179);

        let mut out = Vec::new();
        let mut buf = Vec::new();
        let mut source = self.source.lock().expect("star catalogue lock");
        for row in row_lo..=row_hi {
            // The widest the cone is in RA anywhere in this row's band.
            let band_lo = (row as f64 - 90.0).to_radians();
            let band_hi = band_lo + 1f64.to_radians();
            let max_abs_dec = band_lo.abs().max(band_hi.abs());
            let reaches_pole = dec_c + search >= std::f64::consts::FRAC_PI_2
                || dec_c - search <= -std::f64::consts::FRAC_PI_2;
            let half_width = if reaches_pole || max_abs_dec >= std::f64::consts::FRAC_PI_2 {
                std::f64::consts::PI
            } else {
                let s = search.sin() / max_abs_dec.cos();
                if s >= 1.0 {
                    std::f64::consts::PI
                } else {
                    s.asin()
                }
            };
            for (col_a, col_b) in ra_spans(ra_c, half_width) {
                let first = self.offsets[row * 360 + col_a] as usize;
                let last = self.offsets[row * 360 + col_b + 1] as usize;
                if last == first {
                    continue;
                }
                buf.resize((last - first) * RECORD_BYTES, 0);
                source.seek(SeekFrom::Start(
                    HEADER_BYTES + (first * RECORD_BYTES) as u64,
                ))?;
                source.read_exact(&mut buf)?;
                for rec in buf.as_chunks::<RECORD_BYTES>().0 {
                    let star = decode(rec, years);
                    if star.v_mag <= mag_limit && star.unit_icrf.dot(&center) >= cos_radius {
                        out.push(star);
                    }
                }
            }
        }
        out.sort_by(|a, b| a.v_mag.total_cmp(&b.v_mag));
        Ok(out)
    }
}

/// The inclusive column ranges `[a, b]` (whole degrees of RA) covering
/// `ra_c ± half_width`, split where it wraps through 0h.
fn ra_spans(ra_c: f64, half_width: f64) -> Vec<(usize, usize)> {
    if half_width >= std::f64::consts::PI {
        return vec![(0, 359)];
    }
    let lo = (ra_c - half_width).to_degrees();
    let hi = (ra_c + half_width).to_degrees();
    let col = |deg: f64| (deg.rem_euclid(360.0).floor() as usize).min(359);
    let (a, b) = (col(lo), col(hi));
    if lo < 0.0 || hi >= 360.0 || a > b {
        vec![(a, 359), (0, b)]
    } else {
        vec![(a, b)]
    }
}

fn decode(rec: &[u8], years: f64) -> Star {
    let u32_at = |i: usize| u32::from_le_bytes(rec[i..i + 4].try_into().unwrap());
    let f32_at = |i: usize| f32::from_le_bytes(rec[i..i + 4].try_into().unwrap());
    let u16_at = |i: usize| u16::from_le_bytes(rec[i..i + 2].try_into().unwrap());
    let ra0 = (u32_at(0) as f64 * 1e-7).to_radians();
    let dec0 = (u32_at(4) as i32 as f64 * 1e-7).to_radians();
    let (pm_ra, pm_dec) = (f32_at(8) as f64, f32_at(12) as f64);
    let v_mag = i16::from_le_bytes(rec[16..18].try_into().unwrap()) as f32 / 1000.0;
    let id = TycId(u16_at(18), u16_at(20), rec[22]);
    let flags = rec[23];

    let p0 = unit_of(ra0, dec0);
    let unit = if pm_ra == 0.0 && pm_dec == 0.0 {
        p0
    } else {
        // Along the local east and north directions — the same construction the
        // packer used to carry supplement stars to J2000.
        let east = Vector3::new(-ra0.sin(), ra0.cos(), 0.0);
        let north = Vector3::new(-dec0.sin() * ra0.cos(), -dec0.sin() * ra0.sin(), dec0.cos());
        (p0 + (east * pm_ra + north * pm_dec) * (MAS_TO_RAD * years)).normalize()
    };
    let (ra_rad, dec_rad) = radec_of(&unit);
    Star {
        id,
        unit_icrf: unit,
        ra_rad,
        dec_rad,
        v_mag,
        from_supplement: flags & 1 != 0,
        no_proper_motion: flags & 2 != 0,
    }
}

fn parse_names(text: &str) -> Result<HashMap<TycId, StarName>, StarCatalogError> {
    let mut lines = text.lines();
    if lines.next() != Some("# asteroid-star-names 1") {
        return Err(StarCatalogError::Format("names.txt has no header".into()));
    }
    let mut out = HashMap::new();
    for line in lines.filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split('|').collect();
        let id: Vec<&str> = f[0].split('-').collect();
        if f.len() != 5 || id.len() != 3 {
            return Err(StarCatalogError::Format(format!("names.txt row: {line}")));
        }
        let parse = |s: &str| {
            s.parse::<u32>()
                .map_err(|_| StarCatalogError::Format(format!("names.txt id: {line}")))
        };
        let tyc = TycId(
            parse(id[0])? as u16,
            parse(id[1])? as u16,
            parse(id[2])? as u8,
        );
        out.insert(
            tyc,
            StarName {
                bayer: f[1].to_string(),
                flamsteed: f[2].to_string(),
                constellation: f[3].to_string(),
                proper: f[4].to_string(),
            },
        );
    }
    Ok(out)
}

/// The packed catalogue: [`ENV_STARS`] if set, otherwise `stars/tycho2.stars`
/// in the first kernel directory that has one.
pub fn resolve() -> Option<PathBuf> {
    if let Ok(p) = std::env::var(ENV_STARS) {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    crate::kernels::search_dirs()
        .into_iter()
        .map(|d| d.join("stars").join("tycho2.stars"))
        .find(|p| p.is_file())
}

/// [`resolve`] for a test: `None` (skip) normally, but a **panic** under
/// `ASTEROID_REQUIRE_KERNELS=1`, so a suite that skipped the star checks cannot
/// print green.
pub fn resolve_for_test(what: &str) -> Option<PathBuf> {
    let found = resolve();
    if found.is_none() {
        if crate::kernels::require_kernels() {
            panic!(
                "{what}: the Tycho-2 star catalogue is required but was not found. \
                 Run `python tools/fetch_tycho2.py` (writes kernels/stars/), or set {ENV_STARS}."
            );
        }
        eprintln!("{what}: skipped — no star catalogue (python tools/fetch_tycho2.py)");
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    /// A tiny packed file built the way `fetch_tycho2.py` builds one.
    fn packed(stars: &[(f64, f64, f32, f32, f32, TycId, u8)]) -> Vec<u8> {
        let cell = |ra: f64, dec: f64| {
            ((dec + 90.0).floor() as usize).min(179) * 360 + (ra.floor() as usize).min(359)
        };
        let mut s = stars.to_vec();
        s.sort_by_key(|x| cell(x.0, x.1));
        let mut counts = vec![0u32; CELLS];
        for x in &s {
            counts[cell(x.0, x.1)] += 1;
        }
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&VERSION.to_le_bytes());
        out.extend_from_slice(&(s.len() as u32).to_le_bytes());
        out.extend_from_slice(&1000f32.to_le_bytes());
        let mut acc = 0u32;
        out.extend_from_slice(&acc.to_le_bytes());
        for c in counts {
            acc += c;
            out.extend_from_slice(&acc.to_le_bytes());
        }
        for &(ra, dec, pmra, pmde, v, id, flags) in &s {
            out.extend_from_slice(&((ra * 1e7).round() as u32).to_le_bytes());
            out.extend_from_slice(&((dec * 1e7).round() as i32).to_le_bytes());
            out.extend_from_slice(&pmra.to_le_bytes());
            out.extend_from_slice(&pmde.to_le_bytes());
            out.extend_from_slice(&((v * 1000.0).round() as i16).to_le_bytes());
            out.extend_from_slice(&id.0.to_le_bytes());
            out.extend_from_slice(&id.1.to_le_bytes());
            out.push(id.2);
            out.push(flags);
        }
        out
    }

    fn catalog(stars: &[(f64, f64, f32, f32, f32, TycId, u8)]) -> StarCatalog {
        StarCatalog::from_source(Box::new(Cursor::new(packed(stars)))).unwrap()
    }

    fn ids(v: &[Star]) -> Vec<u16> {
        let mut x: Vec<u16> = v.iter().map(|s| s.id.1).collect();
        x.sort();
        x
    }

    #[test]
    fn a_cone_through_0h_finds_stars_on_both_sides() {
        let cat = catalog(&[
            (359.8, 10.0, 0.0, 0.0, 8.0, TycId(1, 1, 1), 0),
            (0.2, 10.1, 0.0, 0.0, 9.0, TycId(1, 2, 1), 0),
            (1.5, 10.0, 0.0, 0.0, 9.0, TycId(1, 3, 1), 0), // outside 0.5°
            (180.0, 10.0, 0.0, 0.0, 9.0, TycId(1, 4, 1), 0),
        ]);
        let c = unit_of(0.0, 10f64.to_radians());
        let got = cat.cone(&c, 0.5f64.to_radians(), 12.0, 0.0).unwrap();
        assert_eq!(ids(&got), vec![1, 2]);
        assert_eq!(got[0].id.1, 1, "brightest first");
    }

    #[test]
    fn a_cone_over_the_pole_takes_every_ra() {
        let cat = catalog(&[
            (10.0, 89.8, 0.0, 0.0, 5.0, TycId(2, 1, 1), 0),
            (190.0, 89.8, 0.0, 0.0, 5.0, TycId(2, 2, 1), 0),
            (100.0, 88.0, 0.0, 0.0, 5.0, TycId(2, 3, 1), 0), // 2° from the pole
        ]);
        let pole = Vector3::new(0.0, 0.0, 1.0);
        let got = cat.cone(&pole, 0.5f64.to_radians(), 12.0, 0.0).unwrap();
        assert_eq!(ids(&got), vec![1, 2]);
    }

    #[test]
    fn the_magnitude_limit_and_proper_motion_apply() {
        // 1000 mas/yr north for 36 years = 36″ = 0.01°.
        let cat = catalog(&[
            (50.0, 20.0, 0.0, 1000.0, 7.0, TycId(3, 1, 1), 0),
            (50.0, 20.0, 0.0, 0.0, 12.5, TycId(3, 2, 1), 0),
        ]);
        let c = unit_of(50f64.to_radians(), 20f64.to_radians());
        let t = 36.0 * JULIAN_YEAR_S;
        let got = cat.cone(&c, 0.1f64.to_radians(), 12.0, t).unwrap();
        assert_eq!(ids(&got), vec![1]);
        let moved_arcsec = (got[0].dec_rad.to_degrees() - 20.0) * 3600.0;
        assert!((moved_arcsec - 36.0).abs() < 1e-3, "{moved_arcsec}");
    }

    #[test]
    fn names_read_as_labels() {
        let names = parse_names(
            "# asteroid-star-names 1\n3105-2070-1|alf|3|Lyr|Vega\n1-2-1|eta02||Dor|\n5-6-1||21|And|\n",
        )
        .unwrap();
        assert_eq!(names[&TycId(3105, 2070, 1)].label(), "Vega");
        assert_eq!(names[&TycId(1, 2, 1)].label(), "η² Dor");
        assert_eq!(names[&TycId(5, 6, 1)].label(), "21 And");
        assert_eq!(greek("the"), "θ");
        assert_eq!(greek("mu."), "μ");
        assert_eq!(greek("pi.02"), "π²");
    }

    #[test]
    fn a_wrong_file_is_rejected() {
        let err = StarCatalog::from_source(Box::new(Cursor::new(
            b"not a catalogue at all, sorry".to_vec(),
        )));
        assert!(matches!(err, Err(StarCatalogError::Format(_))));
    }
}
