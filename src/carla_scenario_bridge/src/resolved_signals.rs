//! The resolved traffic light table: what acb_bridge needs to publish CARLA's signal state to
//! Autoware (roadmap 015, "The lanelet <-> CARLA light mapping is produced by csb and consumed
//! by acb through a file").
//!
//! This bridge already resolves which CARLA light is which Lanelet2 signal (the explicit
//! `config/traffic_lights_<town>.yaml` merged with the position match). At Initialize it
//! writes that result next to the Lanelet2 map as `traffic_lights.resolved.yaml`:
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
//! The directory is the one that holds the map file itself, symlinks resolved: scenarios name
//! the map through `csb_launch`'s install share, whose files are symlinks into `data/`, while
//! the ego stack's `map_path` names `data/` directly. Writing beside the real file puts the
//! table where both spellings of the map lead. A read-only map directory falls back to
//! [`fallback_dir`], and the path is logged either way.
//!
//! The write is atomic (temporary file, then rename): acb re-reads the file when its mtime
//! changes and must never see half of it.

use std::path::{Path, PathBuf};

use eyre::{Result, WrapErr};
use serde::Serialize;

use crate::lanelet_map::TrafficLightElement;
use crate::traffic_light_mapper::{CarlaSignal, SignalMap};

/// File name of the table, in the map directory.
pub const RESOLVED_FILE_NAME: &str = "traffic_lights.resolved.yaml";

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
            "# Written by carla_scenario_bridge at every Initialize; read by acb_bridge\n\
             # (traffic_light_map_path). Regenerated, do not edit: change\n\
             # config/traffic_lights_<town>.yaml instead. See roadmap 015.\n{body}"
        ))
    }
}

/// The directory the table belongs in: the map file's own directory, symlinks resolved.
pub fn map_dir(lanelet2_map_file: &Path) -> PathBuf {
    let real = std::fs::canonicalize(lanelet2_map_file)
        .unwrap_or_else(|_| lanelet2_map_file.to_path_buf());
    real.parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."))
}

/// Where the table goes when the map directory cannot be written:
/// `$XDG_RUNTIME_DIR` (else the system temp dir) `/carla_scenario_bridge/<town>/`.
pub fn fallback_dir(town: &str) -> PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .filter(|p| p.is_dir())
        .unwrap_or_else(std::env::temp_dir)
        .join("carla_scenario_bridge")
        .join(town)
}

/// Write `contents` to `dir/RESOLVED_FILE_NAME` atomically.
pub fn write_atomic(dir: &Path, contents: &str) -> Result<PathBuf> {
    std::fs::create_dir_all(dir).wrap_err_with(|| format!("create {}", dir.display()))?;
    let path = dir.join(RESOLVED_FILE_NAME);
    let tmp = dir.join(format!(".{RESOLVED_FILE_NAME}.{}.tmp", std::process::id()));
    std::fs::write(&tmp, contents).wrap_err_with(|| format!("write {}", tmp.display()))?;
    if let Err(e) = std::fs::rename(&tmp, &path) {
        let _ = std::fs::remove_file(&tmp);
        return Err(e).wrap_err_with(|| format!("rename onto {}", path.display()));
    }
    Ok(path)
}

/// Where the table was written.
#[derive(Debug, PartialEq)]
pub enum Written {
    MapDir(PathBuf),
    /// The map directory refused it (the error), so it went to the fallback.
    Fallback(PathBuf, String),
}

/// Write to `primary`, or to `fallback` if `primary` cannot be written.
pub fn write_with_fallback(primary: &Path, fallback: &Path, contents: &str) -> Result<Written> {
    match write_atomic(primary, contents) {
        Ok(p) => Ok(Written::MapDir(p)),
        Err(e) => {
            let p = write_atomic(fallback, contents)?;
            Ok(Written::Fallback(p, format!("{e:#}")))
        }
    }
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
    fn the_map_dir_resolves_symlinks() {
        let base = std::env::temp_dir().join(format!("csb_resolved_{}", std::process::id()));
        let real = base.join("data/Town01");
        let share = base.join("share/Town01");
        std::fs::create_dir_all(&real).unwrap();
        std::fs::create_dir_all(&share).unwrap();
        std::fs::write(real.join("lanelet2_map.osm"), "<osm/>").unwrap();
        let link = share.join("lanelet2_map.osm");
        let _ = std::fs::remove_file(&link);
        std::os::unix::fs::symlink(real.join("lanelet2_map.osm"), &link).unwrap();
        assert_eq!(map_dir(&link), std::fs::canonicalize(&real).unwrap());
        let _ = std::fs::remove_dir_all(&base);
    }

    #[test]
    fn a_read_only_map_dir_falls_back() {
        let base = std::env::temp_dir().join(format!("csb_resolved_ro_{}", std::process::id()));
        // A path under a regular file cannot be created, whoever runs the test (root too).
        std::fs::create_dir_all(&base).unwrap();
        let blocker = base.join("not_a_dir");
        std::fs::write(&blocker, "").unwrap();
        let fallback = base.join("fallback");
        match write_with_fallback(&blocker.join("Town01"), &fallback, "x: 1\n").unwrap() {
            Written::Fallback(p, why) => {
                assert_eq!(p, fallback.join(RESOLVED_FILE_NAME));
                assert_eq!(std::fs::read_to_string(&p).unwrap(), "x: 1\n");
                assert!(!why.is_empty());
            }
            other => panic!("expected the fallback, got {other:?}"),
        }
        match write_with_fallback(&base.join("ok"), &fallback, "x: 2\n").unwrap() {
            Written::MapDir(p) => assert_eq!(std::fs::read_to_string(p).unwrap(), "x: 2\n"),
            other => panic!("expected the map dir, got {other:?}"),
        }
        let _ = std::fs::remove_dir_all(&base);
    }
}
