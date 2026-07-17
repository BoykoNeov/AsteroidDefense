class_name Kernels
extends RefCounted
## Finds the JPL DE ephemeris kernels on this machine.
##
## The Rust core and its tests take kernel paths from the ASTEROID_DE_KERNEL /
## ASTEROID_PLANETARY_CONSTANTS environment variables, which is fine for a
## developer shell. A *launched game* is a different story: those variables are
## not persisted at user or machine level, so a double-clicked build (or an
## editor Play, or an MCP-driven run) inherits neither and every body lookup
## fails. Without a resolver the whole display is blank — see Mission.load_from,
## which exists for exactly this.
##
## Kernels are large (de440s.bsp is ~32 MB, sb441-n16.bsp ~646 MB) and are not
## in the repository, so a fresh clone has to be told where they are. Order of
## resolution, first hit wins:
##
##   1. ASTEROID_DE_KERNEL + ASTEROID_PLANETARY_CONSTANTS  (explicit file paths)
##   2. user://kernels.cfg                                 (machine-local config)
##   3. a short list of conventional directories           (see _search_dirs)
##
## Nothing here is committed to project.godot on purpose: that file is in git and
## a kernel path is machine-local absolute, so it would be noise at best and a
## leaked local directory layout at worst.

## Accepted DE kernel filenames, most-preferred first. de440s is the standard
## short span (~1850-2149); de441 is the long span (~1550-2650). The core copes
## with either — Mission.usable_span_tdb() reports whichever is mounted — so this
## is a preference order, not a requirement.
const BSP_NAMES: Array[String] = ["de440s.bsp", "de440.bsp", "de441.bsp"]

## Accepted planetary-constants (GM) filenames. pck11 is what the core tests pin.
const PCA_NAMES: Array[String] = ["pck11.pca"]

const CFG_PATH := "user://kernels.cfg"


## Resolve the kernel pair. Returns
##   {ok: bool, bsp: String, pca: String, source: String, error: String}
## with `source` naming where the hit came from (for the HUD/log — when this goes
## wrong, *which* of three mechanisms answered is the first thing you want to
## know) and `error` describing every place searched when it does not.
static func resolve() -> Dictionary:
	var env := _from_env()
	if not env.is_empty():
		return _ok(env[0], env[1], "ASTEROID_DE_KERNEL env")

	var cfg := _from_config()
	if not cfg.is_empty():
		return _ok(cfg[0], cfg[1], CFG_PATH)

	for dir in _search_dirs():
		var pair := _scan_dir(dir)
		if not pair.is_empty():
			return _ok(pair[0], pair[1], dir)

	return {
		"ok": false, "bsp": "", "pca": "", "source": "",
		"error": _not_found_message(),
	}


## Write the config file that resolve() reads second, so a machine can be taught
## its kernel location once instead of through the environment. Returns "" on
## success or the failure reason. (Nothing calls this yet; it is the documented
## repair path the offline banner points at.)
static func write_config(bsp: String, pca: String) -> String:
	var cfg := ConfigFile.new()
	cfg.set_value("kernels", "de_bsp", bsp)
	cfg.set_value("kernels", "pca", pca)
	var err := cfg.save(CFG_PATH)
	if err != OK:
		return "could not write %s (error %d)" % [CFG_PATH, err]
	return ""


# ------------------------------------------------------------- resolution ---

## [bsp, pca] from the environment, or [] if either is unset/missing on disk.
## Both must be present: half a pair is a misconfiguration worth falling through
## rather than silently pairing an env kernel with a scanned one.
static func _from_env() -> Array:
	var bsp := OS.get_environment("ASTEROID_DE_KERNEL")
	var pca := OS.get_environment("ASTEROID_PLANETARY_CONSTANTS")
	if bsp.is_empty() or pca.is_empty():
		return []
	if not FileAccess.file_exists(bsp) or not FileAccess.file_exists(pca):
		return []
	return [bsp, pca]


## [bsp, pca] from user://kernels.cfg. Accepts either explicit file paths
## (de_bsp/pca) or a directory to scan (dir), whichever the file carries.
static func _from_config() -> Array:
	if not FileAccess.file_exists(CFG_PATH):
		return []
	var cfg := ConfigFile.new()
	if cfg.load(CFG_PATH) != OK:
		return []
	var bsp: String = cfg.get_value("kernels", "de_bsp", "")
	var pca: String = cfg.get_value("kernels", "pca", "")
	if not bsp.is_empty() and not pca.is_empty() \
			and FileAccess.file_exists(bsp) and FileAccess.file_exists(pca):
		return [bsp, pca]
	var dir: String = cfg.get_value("kernels", "dir", "")
	if not dir.is_empty():
		return _scan_dir(dir)
	return []


## Conventional kernel directories, in order. Absolute native paths — res:// is
## globalized because the kernels are ordinary files on disk, not project
## resources, and FileAccess needs a real path to reach outside the project.
static func _search_dirs() -> Array[String]:
	var dirs: Array[String] = []
	# A kernels/ folder inside the project — the natural "drop them here" spot
	# for a fresh clone (gitignored; they are far too large to commit).
	dirs.append(ProjectSettings.globalize_path("res://kernels"))
	# Beside the executable — where an exported build would ship them, since
	# res:// lives inside the .pck there and cannot hold a 32 MB kernel usefully.
	dirs.append(OS.get_executable_path().get_base_dir().path_join("kernels"))
	# This project's conventional scratch root (../temp relative to the repo),
	# which is where the dev machine's kernels actually live.
	dirs.append(ProjectSettings.globalize_path("res://../../temp/AsteroidDefense/kernels"))
	return dirs


## First [bsp, pca] pair found in `dir`, or [] if the directory lacks either.
static func _scan_dir(dir: String) -> Array:
	if dir.is_empty() or not DirAccess.dir_exists_absolute(dir):
		return []
	var bsp := _first_present(dir, BSP_NAMES)
	var pca := _first_present(dir, PCA_NAMES)
	if bsp.is_empty() or pca.is_empty():
		return []
	return [bsp, pca]


static func _first_present(dir: String, names: Array[String]) -> String:
	for n in names:
		var p := dir.path_join(n)
		if FileAccess.file_exists(p):
			return p
	return ""


static func _ok(bsp: String, pca: String, source: String) -> Dictionary:
	return {"ok": true, "bsp": bsp, "pca": pca, "source": source, "error": ""}


## The message the operator actually needs when nothing resolved: every place
## looked, and the two ways to fix it. A bare "kernels not found" would send
## someone hunting through source for the search order.
static func _not_found_message() -> String:
	var lines := PackedStringArray()
	lines.append("no DE kernel found (need one of %s + %s)" %
		[", ".join(BSP_NAMES), ", ".join(PCA_NAMES)])
	lines.append("searched: ASTEROID_DE_KERNEL env, " + CFG_PATH)
	for d in _search_dirs():
		lines.append("searched: " + d)
	lines.append("fix: put the kernels in one of the above, or write " + CFG_PATH +
		' with [kernels] dir="/path/to/kernels"')
	return "\n".join(lines)
