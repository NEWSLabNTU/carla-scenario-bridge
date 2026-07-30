use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityType {
    Ego,
    Vehicle,
    Pedestrian,
    MiscObject,
}

/// What the bridge knows about one SSv2 entity.
///
/// The entity's name is the map key in [`EntityManager`], so it is deliberately not repeated
/// here -- two copies of the same string invite them to disagree.
#[derive(Debug)]
pub struct Entity {
    pub entity_type: EntityType,
    pub carla_actor_id: u32,
}

#[derive(Debug, Default)]
pub struct EntityManager {
    entities: HashMap<String, Entity>,
}

impl EntityManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, name: String, entity_type: EntityType, carla_actor_id: u32) {
        self.entities.insert(
            name,
            Entity {
                entity_type,
                carla_actor_id,
            },
        );
    }

    pub fn remove(&mut self, name: &str) -> Option<u32> {
        self.entities.remove(name).map(|e| e.carla_actor_id)
    }

    pub fn get(&self, name: &str) -> Option<&Entity> {
        self.entities.get(name)
    }

    pub fn clear(&mut self) {
        self.entities.clear();
    }
}
