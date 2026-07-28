//! Traffic light state conversion and lanelet-to-CARLA signal mapping.
//!
//! SSv2 is authoritative for signal state (invariant 3): CARLA's built-in cycling is frozen
//! and every state change comes from an `UpdateTrafficLights` request. This module owns the
//! two translation problems that sit between the two systems.
//!
//! # Identity
//!
//! `TrafficSignal.id` is a Lanelet2 element ID. CARLA's traffic lights come from the
//! OpenDRIVE the map was generated from and know nothing about Lanelet2. The two ID spaces
//! do not overlap, so a mapping is required.
//!
//! The mapping is keyed on **OpenDRIVE sign ID, not CARLA actor ID**. Actor IDs are assigned
//! at spawn and differ every session, so a file mapping lanelet IDs to actor IDs would be
//! stale the moment CARLA restarts. OpenDRIVE sign IDs come from the map and are stable, and
//! `World::traffic_light_from_open_drive` resolves one to a live actor at runtime.
//!
//! # State
//!
//! A Lanelet2 signal carries a list of bulbs, each with a colour, a shape and a status.
//! CARLA models a whole light as one of `Red`/`Yellow`/`Green`/`Off`. Reducing the former to
//! the latter loses the arrows; see [`carla_state_for_signal`].

use std::collections::HashMap;
use std::path::Path;

use carla::rpc::TrafficLightState;
use eyre::{Result, WrapErr};
use serde::Deserialize;

use crate::proto::simulation_api_schema::{traffic_light, TrafficLight, TrafficSignal};

/// Lanelet2 signal ID to OpenDRIVE sign ID, loaded from a per-map file.
///
/// The file lives at `config/traffic_lights_<town>.yaml`:
///
/// ```yaml
/// signals:
///   "1234": "45"   # Lanelet2 element id: OpenDRIVE sign id
///   "1235": "46"
/// ```
#[derive(Debug, Default, Deserialize)]
pub struct SignalMap {
    /// Keys are Lanelet2 IDs as strings, because YAML mappings key on strings.
    #[serde(default)]
    signals: HashMap<String, String>,
}

impl SignalMap {
    pub fn new() -> Self {
        Self::default()
    }

    /// Load a per-map signal mapping, or return an empty map if the file does not exist.
    ///
    /// A missing file is not an error: not every map has traffic lights, and a scenario that
    /// never sends `UpdateTrafficLights` needs no mapping at all. An unmapped signal is
    /// reported when it is used, which is the point at which it matters.
    pub fn load_or_empty(path: &Path) -> Result<Self> {
        if !path.exists() {
            tracing::info!(
                "No traffic light mapping at {}; signals will be unmapped until one exists",
                path.display()
            );
            return Ok(Self::new());
        }

        let text =
            std::fs::read_to_string(path).wrap_err_with(|| format!("read {}", path.display()))?;
        let map: SignalMap =
            serde_yaml::from_str(&text).wrap_err_with(|| format!("parse {}", path.display()))?;

        tracing::info!(
            "Loaded {} traffic light mapping(s) from {}",
            map.len(),
            path.display()
        );
        Ok(map)
    }

    /// OpenDRIVE sign ID for a Lanelet2 signal ID.
    pub fn opendrive_id(&self, lanelet_id: i32) -> Option<&str> {
        self.signals
            .get(&lanelet_id.to_string())
            .map(String::as_str)
    }

    pub fn len(&self) -> usize {
        self.signals.len()
    }

    pub fn is_empty(&self) -> bool {
        self.signals.is_empty()
    }

    #[cfg(test)]
    pub fn from_pairs(pairs: &[(&str, &str)]) -> Self {
        Self {
            signals: pairs
                .iter()
                .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
                .collect(),
        }
    }
}

/// Whether a bulb is emitting light at all.
///
/// `FLASHING` counts as lit. A flashing amber is meaningfully amber, and CARLA cannot render
/// the flashing itself, so treating it as off would understate the signal.
fn is_lit(bulb: &TrafficLight) -> bool {
    matches!(
        traffic_light::Status::try_from(bulb.status),
        Ok(traffic_light::Status::SolidOn) | Ok(traffic_light::Status::Flashing)
    )
}

/// CARLA state for a whole Lanelet2 signal.
///
/// # Reduction rules
///
/// A signal has several bulbs and CARLA has one state, so the bulbs are reduced by
/// **most restrictive wins**: any lit red makes the light red, then amber, then green. An
/// unlit signal, or one whose bulbs are all non-vehicle colours, is `Off`.
///
/// Erring towards restrictive is deliberate. A signal showing a red circle and a green left
/// arrow is a red light to everything except a left-turning vehicle; reporting green would
/// invite the ego through a red.
///
/// # Known limitation: arrows are lost
///
/// CARLA has no arrow states, so `LEFT_ARROW`, `UP_ARROW` and the rest collapse into their
/// colour. A protected-turn scenario cannot be represented faithfully -- the ego sees a plain
/// red or green. Recorded in `docs/roadmap/009-map-and-traffic-lights.md`.
pub fn carla_state_for_signal(signal: &TrafficSignal) -> TrafficLightState {
    let mut saw_amber = false;
    let mut saw_green = false;

    for bulb in &signal.traffic_light_status {
        if !is_lit(bulb) {
            continue;
        }

        match traffic_light::Color::try_from(bulb.color) {
            Ok(traffic_light::Color::Red) => return TrafficLightState::Red,
            Ok(traffic_light::Color::Amber) => saw_amber = true,
            Ok(traffic_light::Color::Green) => saw_green = true,
            // WHITE is a tram/bus signal and UNKNOWN_COLOR is exactly that. Neither maps
            // onto a CARLA vehicle-light state, so they contribute nothing.
            _ => {}
        }
    }

    if saw_amber {
        TrafficLightState::Yellow
    } else if saw_green {
        TrafficLightState::Green
    } else {
        TrafficLightState::Off
    }
}

/// Whether any bulb uses an arrow shape, so the caller can report once that the arrow is
/// being flattened into a plain colour.
pub fn signal_uses_arrows(signal: &TrafficSignal) -> bool {
    signal.traffic_light_status.iter().any(|bulb| {
        !matches!(
            traffic_light::Shape::try_from(bulb.shape),
            Ok(traffic_light::Shape::Circle) | Ok(traffic_light::Shape::UnknownShape)
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `TrafficLightState` has no `Debug` impl, so name it for assertion messages.
    fn state_name(s: TrafficLightState) -> &'static str {
        if s == TrafficLightState::Red {
            "Red"
        } else if s == TrafficLightState::Yellow {
            "Yellow"
        } else if s == TrafficLightState::Green {
            "Green"
        } else if s == TrafficLightState::Off {
            "Off"
        } else {
            "other"
        }
    }

    /// State for a signal built from the given bulbs, named for readable failures.
    fn state_of(bulbs: Vec<TrafficLight>) -> &'static str {
        state_name(carla_state_for_signal(&signal(bulbs)))
    }

    fn bulb(color: traffic_light::Color, status: traffic_light::Status) -> TrafficLight {
        TrafficLight {
            color: color as i32,
            shape: traffic_light::Shape::Circle as i32,
            status: status as i32,
            confidence: 1.0,
        }
    }

    fn shaped_bulb(shape: traffic_light::Shape) -> TrafficLight {
        TrafficLight {
            color: traffic_light::Color::Green as i32,
            shape: shape as i32,
            status: traffic_light::Status::SolidOn as i32,
            confidence: 1.0,
        }
    }

    fn signal(bulbs: Vec<TrafficLight>) -> TrafficSignal {
        TrafficSignal {
            id: 1,
            traffic_light_status: bulbs,
            relation_ids: Vec::new(),
        }
    }

    #[test]
    fn a_lit_bulb_maps_to_its_colour() {
        use traffic_light::{Color, Status};
        assert_eq!(state_of(vec![bulb(Color::Red, Status::SolidOn)]), "Red");
        assert_eq!(
            state_of(vec![bulb(Color::Amber, Status::SolidOn)]),
            "Yellow"
        );
        assert_eq!(state_of(vec![bulb(Color::Green, Status::SolidOn)]), "Green");
    }

    #[test]
    fn an_unlit_signal_is_off() {
        use traffic_light::{Color, Status};
        assert_eq!(state_of(vec![bulb(Color::Green, Status::SolidOff)]), "Off");
        assert_eq!(state_of(vec![]), "Off");
    }

    /// The safety-relevant case: a red circle with a green arrow must not read as green.
    #[test]
    fn the_most_restrictive_lit_bulb_wins() {
        use traffic_light::{Color, Status};
        assert_eq!(
            state_of(vec![
                bulb(Color::Green, Status::SolidOn),
                bulb(Color::Red, Status::SolidOn),
            ]),
            "Red"
        );
        assert_eq!(
            state_of(vec![
                bulb(Color::Green, Status::SolidOn),
                bulb(Color::Amber, Status::SolidOn),
            ]),
            "Yellow"
        );
    }

    /// An unlit red must not suppress a lit green -- only lit bulbs count.
    #[test]
    fn unlit_bulbs_do_not_contribute() {
        use traffic_light::{Color, Status};
        assert_eq!(
            state_of(vec![
                bulb(Color::Red, Status::SolidOff),
                bulb(Color::Green, Status::SolidOn),
            ]),
            "Green"
        );
    }

    /// Flashing amber is amber, not off -- CARLA cannot flash, but the colour still holds.
    #[test]
    fn flashing_counts_as_lit() {
        use traffic_light::{Color, Status};
        assert_eq!(
            state_of(vec![bulb(Color::Amber, Status::Flashing)]),
            "Yellow"
        );
    }

    /// White is a tram signal; neither it nor an unknown colour maps onto a vehicle light.
    #[test]
    fn non_vehicle_colours_contribute_nothing() {
        use traffic_light::{Color, Status};
        assert_eq!(state_of(vec![bulb(Color::White, Status::SolidOn)]), "Off");
        assert_eq!(
            state_of(vec![bulb(Color::UnknownColor, Status::SolidOn)]),
            "Off"
        );
    }

    #[test]
    fn arrow_shapes_are_detected_so_the_loss_can_be_reported() {
        use traffic_light::{Color, Shape, Status};
        assert!(signal_uses_arrows(&signal(vec![shaped_bulb(
            Shape::LeftArrow
        )])));
        assert!(!signal_uses_arrows(&signal(vec![bulb(
            Color::Red,
            Status::SolidOn
        )])));
    }

    #[test]
    fn signal_map_resolves_lanelet_ids_to_opendrive_ids() {
        let map = SignalMap::from_pairs(&[("1234", "45"), ("1235", "46")]);
        assert_eq!(map.opendrive_id(1234), Some("45"));
        assert_eq!(map.opendrive_id(1235), Some("46"));
        assert_eq!(map.opendrive_id(9999), None);
        assert_eq!(map.len(), 2);
    }

    #[test]
    fn an_empty_signal_map_resolves_nothing() {
        let map = SignalMap::new();
        assert!(map.is_empty());
        assert_eq!(map.opendrive_id(1), None);
    }

    #[test]
    fn signal_map_parses_yaml() {
        let yaml = "signals:\n  \"1234\": \"45\"\n  \"1235\": \"46\"\n";
        let map: SignalMap = serde_yaml::from_str(yaml).expect("parses");
        assert_eq!(map.opendrive_id(1234), Some("45"));
        assert_eq!(map.len(), 2);
    }

    /// A file with no `signals` key is a valid empty mapping, not a parse failure.
    #[test]
    fn signal_map_tolerates_a_missing_section() {
        let map: SignalMap = serde_yaml::from_str("{}").expect("parses");
        assert!(map.is_empty());
    }
}
