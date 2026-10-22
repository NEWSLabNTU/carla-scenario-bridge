//! Resolving a CARLA town name from the map path a scenario asks for.
//!
//! `InitializeRequest.lanelet2_map_path` names the Lanelet2 map the scenario was authored
//! against. CARLA needs a town name. Nothing carries the correspondence explicitly, so it is
//! recovered from the path, which by convention ends in the town directory:
//!
//! ```text
//! .../data/carla-autoware-bridge/Town01                  -> Town01
//! .../data/carla-autoware-bridge/Town01/lanelet2_map.osm -> Town01
//! ```
//!
//! Getting this wrong is not a visible failure. Every pose in the scenario is interpreted
//! against whatever town CARLA happens to hold, so a mismatch produces a run that completes
//! and means nothing. That is why an unresolvable path fails `Initialize` rather than
//! guessing.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Filenames that sit inside a town directory rather than naming it.
///
/// When the path ends in one of these, the town is the parent directory.
const MAP_FILENAMES: &[&str] = &["lanelet2_map.osm", "pointcloud_map.pcd"];

/// Resolve a CARLA town name from a Lanelet2 map path.
///
/// `aliases` maps a directory name to a CARLA town when the two differ. An empty alias table
/// means the directory name is used as-is.
///
/// Returns `None` for a path with no usable final component.
pub fn town_from_map_path(path: &str, aliases: &HashMap<String, String>) -> Option<String> {
    let trimmed = path.trim().trim_end_matches('/');
    if trimmed.is_empty() {
        return None;
    }

    let path = Path::new(trimmed);

    // A path ending in a known map file names the town in its parent directory.
    let candidate = match path.file_name().and_then(|n| n.to_str()) {
        Some(name) if MAP_FILENAMES.contains(&name) || name.ends_with(".osm") => path
            .parent()
            .and_then(|p| p.file_name())
            .and_then(|n| n.to_str())?,
        Some(name) => name,
        None => return None,
    };

    if candidate.is_empty() {
        return None;
    }

    Some(
        aliases
            .get(candidate)
            .cloned()
            .unwrap_or_else(|| candidate.to_string()),
    )
}

/// The Lanelet2 `.osm` file for a map path.
///
/// SSv2 may name either the town directory or the map file itself, so both forms resolve to
/// the same file.
pub fn lanelet_map_file(map_path: &str) -> std::path::PathBuf {
    let path = Path::new(map_path.trim().trim_end_matches('/'));

    let names_a_file = path
        .file_name()
        .and_then(|n| n.to_str())
        .is_some_and(|n| n.ends_with(".osm"));

    if names_a_file {
        path.to_path_buf()
    } else {
        path.join("lanelet2_map.osm")
    }
}

/// The map directory a Lanelet2 map path names: the path itself if it is a directory,
/// else the file's parent.
pub fn map_dir_of(lanelet2_map_path: &Path) -> PathBuf {
    if lanelet2_map_path.is_dir() {
        lanelet2_map_path.to_path_buf()
    } else {
        lanelet2_map_path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."))
    }
}

/// Where a map's signal table lives, relative to its directory (roadmap 046): one table any
/// simulator with signals reads -- per Lanelet2 traffic light way, its regulatory elements,
/// its OpenDRIVE signal and where the lamp stands. CARLA's world server generates it.
pub const SIGNAL_TABLE_SUBPATH: &str = "traffic_lights.yaml";

/// Where it lived before roadmap 046. Still read when the new one is absent, and written as
/// a link to the new one, so a vehicle bridge configured with it (acb's
/// `traffic_light_map_path` default) keeps finding it.
pub const OLD_SIGNAL_TABLE_SUBPATH: &str = "carla/traffic_lights.yaml";

/// The signal table of the map at `map_dir`: the new place, else the old one when only it
/// exists, else the new place (absent).
pub fn signal_table(map_dir: &Path) -> PathBuf {
    let new = map_dir.join(SIGNAL_TABLE_SUBPATH);
    let old = map_dir.join(OLD_SIGNAL_TABLE_SUBPATH);
    if !new.exists() && old.exists() {
        old
    } else {
        new
    }
}

/// Where a map's `map_origin.yaml` lives, relative to its directory.
pub const MAP_ORIGIN_SUBPATH: &str = "carla/map_origin.yaml";

/// The map-frame position of CARLA's world origin for the map in `map_dir`.
///
/// `<map dir>/carla/map_origin.yaml` holds `x`, `y` and optionally `z`, in map-frame metres:
/// the offset the map's OpenDRIVE was converted with. No file means the map is authored in
/// CARLA's frame (origin zero), which is every TUM town. A file that does not parse is an
/// error, not zero: a wrong origin puts every scenario pose hundreds of metres off, on a run
/// that otherwise looks fine.
pub fn load_map_origin(map_dir: &Path) -> eyre::Result<crate::coordinate_conversion::MapOrigin> {
    let path = map_dir.join(MAP_ORIGIN_SUBPATH);
    match std::fs::read_to_string(&path) {
        Ok(text) => serde_yaml::from_str(&text)
            .map_err(|e| eyre::eyre!("cannot parse {}: {e}", path.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Default::default()),
        Err(e) => Err(eyre::eyre!("cannot read {}: {e}", path.display())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_aliases() -> HashMap<String, String> {
        HashMap::new()
    }

    #[test]
    fn map_origin_is_zero_without_a_file_and_read_from_one() {
        use crate::coordinate_conversion::MapOrigin;
        let dir = std::env::temp_dir().join(format!("swb-map-origin-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("carla")).unwrap();
        assert_eq!(load_map_origin(&dir).unwrap(), MapOrigin::default());

        std::fs::write(dir.join(MAP_ORIGIN_SUBPATH), "x: 52600.0\ny: 67800.0\n").unwrap();
        assert_eq!(
            load_map_origin(&dir).unwrap(),
            MapOrigin {
                x: 52600.0,
                y: 67800.0,
                z: 0.0
            }
        );

        std::fs::write(dir.join(MAP_ORIGIN_SUBPATH), "x: 1\ny: 2\nyaw: 0.1\n").unwrap();
        assert!(
            load_map_origin(&dir).is_err(),
            "a yaw it cannot apply is refused"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_directory_gains_the_conventional_map_filename() {
        assert_eq!(
            lanelet_map_file("/data/Town01"),
            Path::new("/data/Town01/lanelet2_map.osm")
        );
        assert_eq!(
            lanelet_map_file("/data/Town01/"),
            Path::new("/data/Town01/lanelet2_map.osm")
        );
    }

    #[test]
    fn an_osm_path_is_used_as_is() {
        assert_eq!(
            lanelet_map_file("/data/Town01/lanelet2_map.osm"),
            Path::new("/data/Town01/lanelet2_map.osm")
        );
    }

    #[test]
    fn a_town_directory_resolves_to_itself() {
        assert_eq!(
            town_from_map_path("/data/carla-autoware-bridge/Town01", &no_aliases()),
            Some("Town01".to_string())
        );
    }

    /// SSv2 may name the map file rather than the directory holding it.
    #[test]
    fn a_map_file_resolves_to_its_directory() {
        assert_eq!(
            town_from_map_path(
                "/data/carla-autoware-bridge/Town05/lanelet2_map.osm",
                &no_aliases()
            ),
            Some("Town05".to_string())
        );
        assert_eq!(
            town_from_map_path("/maps/Town10/pointcloud_map.pcd", &no_aliases()),
            Some("Town10".to_string())
        );
    }

    /// Any .osm is treated as a map file, not as a town name.
    #[test]
    fn any_osm_file_resolves_to_its_directory() {
        assert_eq!(
            town_from_map_path("/maps/Town03/some_other_name.osm", &no_aliases()),
            Some("Town03".to_string())
        );
    }

    #[test]
    fn a_trailing_slash_is_ignored() {
        assert_eq!(
            town_from_map_path("/data/Town02/", &no_aliases()),
            Some("Town02".to_string())
        );
    }

    #[test]
    fn aliases_override_the_directory_name() {
        let mut aliases = HashMap::new();
        aliases.insert("my_custom_map".to_string(), "Town07".to_string());
        assert_eq!(
            town_from_map_path("/data/my_custom_map", &aliases),
            Some("Town07".to_string())
        );
    }

    #[test]
    fn an_alias_applies_through_a_map_filename_too() {
        let mut aliases = HashMap::new();
        aliases.insert("kashiwanoha".to_string(), "Town01".to_string());
        assert_eq!(
            town_from_map_path("/data/kashiwanoha/lanelet2_map.osm", &aliases),
            Some("Town01".to_string())
        );
    }

    #[test]
    fn an_unusable_path_resolves_to_nothing() {
        assert_eq!(town_from_map_path("", &no_aliases()), None);
        assert_eq!(town_from_map_path("   ", &no_aliases()), None);
        assert_eq!(town_from_map_path("/", &no_aliases()), None);
    }

    /// A bare filename with no directory has no town to fall back to.
    #[test]
    fn a_bare_map_filename_resolves_to_nothing() {
        assert_eq!(town_from_map_path("lanelet2_map.osm", &no_aliases()), None);
    }
}
