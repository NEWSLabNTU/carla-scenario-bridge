/// Traffic Light Mapper: maps SSv2 lanelet signal IDs to CARLA TrafficLight actors.
///
/// SSv2 references traffic lights by lanelet signal ID (from Lanelet2 map).
/// CARLA has its own TrafficLight actors tied to OpenDRIVE signal definitions.
/// This mapper bridges the two via a mapping table that can be:
/// 1. Auto-generated from lane-to-lanelet correspondence (Phase 5)
/// 2. Loaded from a manual mapping file
/// 3. Built at runtime by matching geographic positions

use std::collections::HashMap;

/// Maps lanelet signal IDs to CARLA TrafficLight actor IDs.
#[derive(Debug, Default)]
pub struct TrafficLightMapper {
    /// lanelet signal ID -> CARLA actor ID
    lanelet_to_carla: HashMap<i32, u32>,
}

impl TrafficLightMapper {
    pub fn new() -> Self {
        Self::default()
    }

    /// Look up the CARLA actor ID for a lanelet signal ID.
    pub fn get_carla_actor(&self, lanelet_signal_id: i32) -> Option<u32> {
        self.lanelet_to_carla.get(&lanelet_signal_id).copied()
    }

    /// Register a mapping entry.
    pub fn insert(&mut self, lanelet_signal_id: i32, carla_actor_id: u32) {
        self.lanelet_to_carla
            .insert(lanelet_signal_id, carla_actor_id);
    }

    // TODO(Phase 4): Load mapping from file
    // TODO(Phase 4): Auto-build mapping from CARLA world + OpenDRIVE
    // TODO(Phase 5): Generate from lane-to-lanelet correspondence CSV
}
