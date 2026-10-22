//! Traffic signals on Isaac (roadmap 044 B): the map's signal table for the world server, and
//! SSv2's signal states reduced to the one colour a lamp shows.
//!
//! The table is the map artifact the CARLA backend checks and acb publishes from,
//! `<map dir>/carla/traffic_lights.yaml` (`backend/carla/resolved_signals.rs`): per Lanelet2
//! traffic light way, its regulatory elements, its OpenDRIVE signal and the CARLA actor's
//! position. Read here without CARLA: the world server finds the lamps by OpenDRIVE id (a
//! SimReady export's `omni:simready:signalID`) or, failing that, by position.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::proto::simulation_api_schema::{traffic_light, TrafficSignal};
use crate::proto::swb_world as wire;

/// Where a map's signal table is, under its map directory (`carla/traffic_lights.yaml`
/// before roadmap 046, still read).
pub const TABLE_SUBPATH: &str = crate::map_resolver::SIGNAL_TABLE_SUBPATH;

#[derive(Debug, Clone, Deserialize)]
struct TableFile {
    #[serde(default)]
    signals: Vec<TableEntry>,
}

/// One mapped Lanelet2 traffic light way.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct TableEntry {
    pub way_id: u64,
    #[serde(default)]
    pub regulatory_element_ids: Vec<u64>,
    pub opendrive_id: String,
    /// CARLA frame (left-handed), m.
    #[serde(default)]
    pub carla_position: Option<[f64; 3]>,
}

/// The signal table of a map: its entries, by way id for SSv2's commands.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SignalTable {
    pub entries: Vec<TableEntry>,
    by_way: BTreeMap<u64, usize>,
    /// Where it was read from; `None` when the map has none.
    pub path: Option<PathBuf>,
}

impl SignalTable {
    /// The table beside the map at `map_dir`; empty when there is none (a map without lights,
    /// or one not generated yet). A table that does not parse is an error.
    pub fn for_map_dir(map_dir: &Path) -> eyre::Result<Self> {
        let path = crate::map_resolver::signal_table(map_dir);
        if !path.exists() {
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(&path)
            .map_err(|e| eyre::eyre!("read {}: {e}", path.display()))?;
        let mut table =
            Self::parse(&text).map_err(|e| eyre::eyre!("parse {}: {e}", path.display()))?;
        table.path = Some(path);
        Ok(table)
    }

    pub fn parse(yaml: &str) -> Result<Self, serde_yaml::Error> {
        let file: TableFile = serde_yaml::from_str(yaml)?;
        let by_way = file
            .signals
            .iter()
            .enumerate()
            .map(|(i, e)| (e.way_id, i))
            .collect();
        Ok(Self {
            entries: file.signals,
            by_way,
            path: None,
        })
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The OpenDRIVE signal a way is shown by.
    pub fn opendrive_id(&self, way_id: u64) -> Option<&str> {
        self.by_way
            .get(&way_id)
            .map(|&i| self.entries[i].opendrive_id.as_str())
    }

    /// The table for `LoadWorld`: positions in the world frame (CARLA's Y flipped into the
    /// map frame, less the map origin).
    pub fn wire(&self, map_origin: [f64; 3]) -> Vec<wire::Signal> {
        self.entries
            .iter()
            .map(|e| wire::Signal {
                way_id: e.way_id,
                regulatory_element_ids: e.regulatory_element_ids.clone(),
                opendrive_id: e.opendrive_id.clone(),
                position: e.carla_position.map(|p| wire::Vec3 {
                    x: p[0] - map_origin[0],
                    y: -p[1] - map_origin[1],
                    z: p[2] - map_origin[2],
                }),
            })
            .collect()
    }
}

/// Whether a bulb is lit: on or flashing, as on CARLA.
fn is_lit(status: i32) -> bool {
    matches!(
        traffic_light::Status::try_from(status),
        Ok(traffic_light::Status::SolidOn) | Ok(traffic_light::Status::Flashing)
    )
}

/// The one colour a lamp shows for an SSv2 signal, as the CARLA backend reduces it
/// (`carla_state_for_signal`): a lit red wins, then amber, then green; arrows flatten to
/// their colour; nothing lit is off.
pub fn colour_for_signal(signal: &TrafficSignal) -> wire::Colour {
    let (mut amber, mut green) = (false, false);
    for bulb in &signal.traffic_light_status {
        if !is_lit(bulb.status) {
            continue;
        }
        match traffic_light::Color::try_from(bulb.color) {
            Ok(traffic_light::Color::Red) => return wire::Colour::Red,
            Ok(traffic_light::Color::Amber) => amber = true,
            Ok(traffic_light::Color::Green) => green = true,
            _ => {}
        }
    }
    if amber {
        wire::Colour::Amber
    } else if green {
        wire::Colour::Green
    } else {
        wire::Colour::Off
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::simulation_api_schema::TrafficLight;

    const TABLE: &str = "town: Town01\nlanelet2_map: /x/lanelet2_map.osm\nsignals:\n\
        - way_id: 43763\n  regulatory_element_ids:\n  - 43856\n  opendrive_id: '377'\n  \
        carla_position:\n  - 102.72\n  - 52.85\n  - 0.10\n\
        - way_id: 43766\n  regulatory_element_ids: [43856, 43900]\n  opendrive_id: '376'\n";

    fn bulb(color: traffic_light::Color, status: traffic_light::Status) -> TrafficLight {
        TrafficLight {
            color: color as i32,
            shape: traffic_light::Shape::Circle as i32,
            status: status as i32,
            confidence: 1.0,
        }
    }

    #[test]
    fn the_table_maps_ways_to_opendrive_and_flips_y_for_the_world() {
        let t = SignalTable::parse(TABLE).unwrap();
        assert_eq!(t.len(), 2);
        assert_eq!(t.opendrive_id(43763), Some("377"));
        assert_eq!(t.opendrive_id(1), None);
        let w = t.wire([0.0, 0.0, 0.0]);
        let p = w[0].position.unwrap();
        assert_eq!((p.x, p.y), (102.72, -52.85));
        assert_eq!(w[1].regulatory_element_ids, vec![43856, 43900]);
        assert!(w[1].position.is_none());
    }

    #[test]
    fn a_map_without_a_table_has_no_signals() {
        let t = SignalTable::for_map_dir(Path::new("/nonexistent/Town99")).unwrap();
        assert!(t.is_empty() && t.path.is_none());
    }

    #[test]
    fn red_wins_then_amber_then_green_and_off_bulbs_do_not_count() {
        use traffic_light::{Color, Status};
        let signal = |bulbs: Vec<TrafficLight>| TrafficSignal {
            id: 43763,
            traffic_light_status: bulbs,
            ..Default::default()
        };
        let s = signal(vec![
            bulb(Color::Green, Status::SolidOn),
            bulb(Color::Red, Status::SolidOn),
        ]);
        assert_eq!(colour_for_signal(&s), wire::Colour::Red);
        let s = signal(vec![
            bulb(Color::Green, Status::SolidOn),
            bulb(Color::Amber, Status::Flashing),
        ]);
        assert_eq!(colour_for_signal(&s), wire::Colour::Amber);
        let s = signal(vec![
            bulb(Color::Red, Status::SolidOff),
            bulb(Color::Green, Status::SolidOn),
        ]);
        assert_eq!(colour_for_signal(&s), wire::Colour::Green);
        assert_eq!(colour_for_signal(&signal(vec![])), wire::Colour::Off);
    }
}
