/// Entity Manager: maps SSv2 entity names to CARLA actor IDs.
///
/// Tracks spawned entities with their type (ego/vehicle/pedestrian/misc),
/// CARLA actor ID, and current status. Handles coordinate conversion
/// between SSv2's ROS frame and CARLA's left-handed frame.

use std::collections::HashMap;

/// Entity type matching SSv2's EntityType enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityType {
    Ego,
    Vehicle,
    Pedestrian,
    MiscObject,
}

/// A tracked entity in the simulation.
#[derive(Debug)]
pub struct Entity {
    pub name: String,
    pub entity_type: EntityType,
    pub carla_actor_id: u32,
}

/// Manages the mapping between SSv2 entity names and CARLA actors.
#[derive(Debug, Default)]
pub struct EntityManager {
    entities: HashMap<String, Entity>,
}

impl EntityManager {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a newly spawned entity.
    pub fn insert(&mut self, name: String, entity_type: EntityType, carla_actor_id: u32) {
        self.entities.insert(
            name.clone(),
            Entity {
                name,
                entity_type,
                carla_actor_id,
            },
        );
    }

    /// Remove a despawned entity. Returns the CARLA actor ID if found.
    pub fn remove(&mut self, name: &str) -> Option<u32> {
        self.entities.remove(name).map(|e| e.carla_actor_id)
    }

    /// Look up an entity by SSv2 name.
    pub fn get(&self, name: &str) -> Option<&Entity> {
        self.entities.get(name)
    }

    /// Get the ego entity, if one exists.
    pub fn ego(&self) -> Option<&Entity> {
        self.entities
            .values()
            .find(|e| e.entity_type == EntityType::Ego)
    }

    /// Iterate over all entities.
    pub fn iter(&self) -> impl Iterator<Item = &Entity> {
        self.entities.values()
    }

    /// Number of tracked entities.
    pub fn len(&self) -> usize {
        self.entities.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entities.is_empty()
    }
}
