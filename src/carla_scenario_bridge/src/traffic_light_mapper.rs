use std::collections::HashMap;

/// Maps lanelet signal IDs to CARLA TrafficLight actor IDs.
/// Phase 4 implementation.
#[derive(Debug, Default)]
pub struct TrafficLightMapper {
    lanelet_to_carla: HashMap<i32, u32>,
}

impl TrafficLightMapper {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn get_carla_actor(&self, lanelet_signal_id: i32) -> Option<u32> {
        self.lanelet_to_carla.get(&lanelet_signal_id).copied()
    }

    pub fn insert(&mut self, lanelet_signal_id: i32, carla_actor_id: u32) {
        self.lanelet_to_carla
            .insert(lanelet_signal_id, carla_actor_id);
    }
}
