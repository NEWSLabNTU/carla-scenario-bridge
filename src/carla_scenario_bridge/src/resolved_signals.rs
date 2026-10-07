//! The resolved traffic light table: what acb_bridge needs to publish CARLA's signal state to
//! Autoware (roadmap 015, "The lanelet <-> CARLA light mapping is produced by csb and consumed
//! by acb through a file").
//!
//! This bridge already resolves which CARLA light is which Lanelet2 signal (the explicit
//! `config/traffic_lights_<town>.yaml` merged with the position match). At Initialize it
//! checks that result against the map artifact `<map dir>/carla/traffic_lights.yaml`:
//!
//! ```yaml
//! town: Town01
//! lanelet2_map: /.../Town01/lanelet2_map.osm
//! signals:                      # one entry per mapped Lanelet2 traffic light way
//!   - way_id: 43763             # what SSv2 commands (TrafficSignal.id)
//!     regulatory_element_ids: [43856]   # what Autoware keys TrafficLightGroup on
//!     opendrive_id: "12"        # World::traffic_light_from_open_drive
//!     carla_position: [92.1, -55.3, 0.0]   # CARLA frame, metres (the actor origin)
//! unmapped:
//!   lanelet_way_ids: []         # Lanelet2 lights with no CARLA light
//!   carla_opendrive_ids: []     # CARLA lights the Lanelet2 map does not reference
//! ```
//!
//! Since roadmap 017 the table is a **map artifact**, generated once per map with
//! `carla_scenario_bridge --generate-signal-table <map dir>` into
//! `<map dir>/carla/traffic_lights.yaml` (CARLA-specific, so in the map dir's `carla/`).
//! At Initialize csb resolves the mapping again and **checks** it against that file instead
//! of writing into the map directory: the map dir is the user's, read-only at run time.
//! A difference fails Initialize, because acb would publish signals for the wrong lights.

use std::path::{Path, PathBuf};

use eyre::{Result, WrapErr};
use serde::Serialize;

use crate::lanelet_map::TrafficLightElement;
use crate::traffic_light_mapper::{CarlaSignal, SignalMap};

/// Where the table lives, relative to the map directory.
pub const TABLE_SUBPATH: &str = "carla/traffic_lights.yaml";

#[derive(Debug, Serialize, PartialEq)]
pub struct ResolvedSignal {
    pub way_id: i32,
    pub regulatory_element_ids: Vec<i64>,
    pub opendrive_id: String,
    /// `None` when the explicit mapping names a sign CARLA does not have.
    pub carla_position: Option<[f64; 3]>,
}

#[derive(Debug, Default, Serialize, PartialEq)]
pub struct Unmapped {
    pub lanelet_way_ids: Vec<i32>,
    pub carla_opendrive_ids: Vec<String>,
}

#[derive(Debug, Serialize, PartialEq)]
pub struct ResolvedTable {
    pub town: String,
    pub lanelet2_map: String,
    pub signals: Vec<ResolvedSignal>,
    pub unmapped: Unmapped,
}

impl ResolvedTable {
    /// Join the mapping with the Lanelet2 lights (for their regulatory elements) and CARLA's
    /// lights (for their positions). Every mapped way is listed, even one whose regulatory
    /// element or CARLA light is unknown, so the gap is visible in the file.
    pub fn build(
        town: &str,
        lanelet2_map: &Path,
        map: &SignalMap,
        lanelet: &[TrafficLightElement],
        carla: &[CarlaSignal],
    ) -> Self {
        let entries = map.entries();
        let signals = entries
            .iter()
            .map(|&(way_id, opendrive_id)| ResolvedSignal {
                way_id,
                regulatory_element_ids: lanelet
                    .iter()
                    .find(|l| l.way_id == way_id)
                    .map(|l| l.regulatory_element_ids.clone())
                    .unwrap_or_default(),
                opendrive_id: opendrive_id.to_string(),
                carla_position: carla
                    .iter()
                    .find(|c| c.opendrive_id == opendrive_id)
                    .map(|c| [c.x, c.y, c.z]),
            })
            .collect();

        let mut lanelet_way_ids: Vec<i32> = lanelet
            .iter()
            .map(|l| l.way_id)
            .filter(|id| !entries.iter().any(|(w, _)| w == id))
            .collect();
        lanelet_way_ids.sort_unstable();
        let mut carla_opendrive_ids: Vec<String> = carla
            .iter()
            .map(|c| c.opendrive_id.clone())
            .filter(|id| !entries.iter().any(|(_, o)| o == id))
            .collect();
        carla_opendrive_ids.sort();

        Self {
            town: town.to_string(),
            lanelet2_map: lanelet2_map.display().to_string(),
            signals,
            unmapped: Unmapped {
                lanelet_way_ids,
                carla_opendrive_ids,
            },
        }
    }

    /// Distinct regulatory elements with at least one mapped light, for the log line.
    pub fn regulatory_element_count(&self) -> usize {
        let mut ids: Vec<i64> = self
            .signals
            .iter()
            .flat_map(|s| s.regulatory_element_ids.iter().copied())
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids.len()
    }

    pub fn to_yaml(&self) -> Result<String> {
        let body = serde_yaml::to_string(self).wrap_err("serialize the resolved table")?;
        Ok(format!(
            "# Generated by carla_scenario_bridge --generate-signal-table; read by acb_bridge\n\
             # (traffic_light_map_path) and checked by csb at every Initialize. Do not edit:\n\
             # correct config/traffic_lights_<town>.yaml and regenerate. Roadmap 015, 017.\n{body}"
        ))
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

pub fn table_path(map_dir: &Path) -> PathBuf {
    map_dir.join(TABLE_SUBPATH)
}

/// Write the table into `map_dir/carla/` atomically (acb re-reads it on change and must
/// never see half of it). Only the generator writes; Initialize checks.
pub fn write_table(map_dir: &Path, contents: &str) -> Result<PathBuf> {
    let path = table_path(map_dir);
    let dir = path.parent().expect("TABLE_SUBPATH has a parent");
    std::fs::create_dir_all(dir).wrap_err_with(|| format!("create {}", dir.display()))?;
    let tmp = dir.join(format!(".traffic_lights.yaml.{}.tmp", std::process::id()));
    std::fs::write(&tmp, contents).wrap_err_with(|| format!("write {}", tmp.display()))?;
    if let Err(e) = std::fs::rename(&tmp, &path) {
        let _ = std::fs::remove_file(&tmp);
        return Err(e).wrap_err_with(|| format!("rename onto {}", path.display()));
    }
    Ok(path)
}

/// How the table on disk compares with the one just resolved.
#[derive(Debug, PartialEq)]
pub enum Check {
    Matches,
    Missing,
    /// The first difference, for the error message.
    Differs(String),
}

/// Compare the file at `path` with `table`, ignoring `lanelet2_map` (the same map is
/// spelled differently wherever it is mounted) and comments.
pub fn check_table(path: &Path, table: &ResolvedTable) -> Result<Check> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Check::Missing),
        Err(e) => return Err(e).wrap_err_with(|| format!("read {}", path.display())),
    };
    let mut on_disk: serde_yaml::Value =
        serde_yaml::from_str(&text).wrap_err_with(|| format!("parse {}", path.display()))?;
    let mut resolved = serde_yaml::to_value(table).wrap_err("serialize the resolved table")?;
    for v in [&mut on_disk, &mut resolved] {
        if let Some(m) = v.as_mapping_mut() {
            m.remove("lanelet2_map");
        }
    }
    if on_disk == resolved {
        return Ok(Check::Matches);
    }
    let field = ["town", "signals", "unmapped"]
        .into_iter()
        .find(|k| on_disk.get(k) != resolved.get(k))
        .unwrap_or("structure");
    Ok(Check::Differs(format!("`{field}` differs")))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn light(way_id: i32, reg: &[i64]) -> TrafficLightElement {
        TrafficLightElement {
            way_id,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            regulatory_element_ids: reg.to_vec(),
        }
    }

    fn carla(id: &str, x: f64, y: f64, z: f64) -> CarlaSignal {
        CarlaSignal {
            opendrive_id: id.to_string(),
            x,
            y,
            z,
        }
    }

    fn table() -> ResolvedTable {
        let map = SignalMap::from_pairs(&[("43763", "12"), ("43760", "13"), ("99", "77")]);
        let lanelet = [
            light(43763, &[43856]),
            light(43760, &[43856]),
            light(500, &[501]),
        ];
        let carla_lights = [
            carla("12", 1.0, -2.0, 0.5),
            carla("13", 3.0, -4.0, 0.5),
            carla("40", 0.0, 0.0, 0.0),
        ];
        ResolvedTable::build(
            "Town01",
            Path::new("/m/Town01/lanelet2_map.osm"),
            &map,
            &lanelet,
            &carla_lights,
        )
    }

    #[test]
    fn every_mapped_way_names_its_regulatory_element_and_carla_light() {
        let t = table();
        assert_eq!(
            t.signals,
            vec![
                ResolvedSignal {
                    way_id: 99,
                    regulatory_element_ids: vec![],
                    opendrive_id: "77".into(),
                    carla_position: None,
                },
                ResolvedSignal {
                    way_id: 43760,
                    regulatory_element_ids: vec![43856],
                    opendrive_id: "13".into(),
                    carla_position: Some([3.0, -4.0, 0.5]),
                },
                ResolvedSignal {
                    way_id: 43763,
                    regulatory_element_ids: vec![43856],
                    opendrive_id: "12".into(),
                    carla_position: Some([1.0, -2.0, 0.5]),
                },
            ]
        );
        assert_eq!(t.regulatory_element_count(), 1);
    }

    #[test]
    fn unmapped_lights_are_listed_on_both_sides() {
        let t = table();
        assert_eq!(t.unmapped.lanelet_way_ids, vec![500]);
        assert_eq!(t.unmapped.carla_opendrive_ids, vec!["40".to_string()]);
    }

    #[test]
    fn the_yaml_round_trips_the_fields_acb_reads() {
        let yaml = table().to_yaml().unwrap();
        let v: serde_yaml::Value = serde_yaml::from_str(&yaml).unwrap();
        let s = &v["signals"][2];
        assert_eq!(s["way_id"].as_i64(), Some(43763));
        assert_eq!(s["regulatory_element_ids"][0].as_i64(), Some(43856));
        assert_eq!(s["opendrive_id"].as_str(), Some("12"));
        assert_eq!(s["carla_position"][1].as_f64(), Some(-2.0));
        assert!(v["signals"][0]["carla_position"].is_null());
    }

    #[test]
    fn generated_table_checks_clean_and_detects_a_change() {
        let base = std::env::temp_dir().join(format!("csb_table_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let path = table_path(&base);
        assert_eq!(check_table(&path, &table()).unwrap(), Check::Missing);

        let written = write_table(&base, &table().to_yaml().unwrap()).unwrap();
        assert_eq!(written, base.join("carla/traffic_lights.yaml"));
        assert_eq!(check_table(&path, &table()).unwrap(), Check::Matches);

        // The map path is spelled differently where it is mounted; that is not a change.
        let mut moved = table();
        moved.lanelet2_map = "/elsewhere/lanelet2_map.osm".into();
        assert_eq!(check_table(&path, &moved).unwrap(), Check::Matches);

        let mut changed = table();
        changed.signals.pop();
        assert!(matches!(check_table(&path, &changed).unwrap(), Check::Differs(d) if d.contains("signals")));
        let _ = std::fs::remove_dir_all(&base);
    }

    #[test]
    fn map_dir_of_a_file_or_a_directory() {
        let base = std::env::temp_dir().join(format!("csb_mapdir_{}", std::process::id()));
        std::fs::create_dir_all(&base).unwrap();
        assert_eq!(map_dir_of(&base), base);
        assert_eq!(map_dir_of(&base.join("lanelet2_map.osm")), base);
        let _ = std::fs::remove_dir_all(&base);
    }
}
