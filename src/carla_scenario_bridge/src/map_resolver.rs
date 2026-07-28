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
use std::path::Path;

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

#[cfg(test)]
mod tests {
    use super::*;

    fn no_aliases() -> HashMap<String, String> {
        HashMap::new()
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
