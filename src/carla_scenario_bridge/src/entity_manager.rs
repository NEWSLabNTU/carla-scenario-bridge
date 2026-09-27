use std::collections::HashMap;

use crate::coordinate_conversion::OriginOffset;

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
    /// The scenario-declared `bounding_box.center` (x, y), fixed at spawn. Every pose that
    /// crosses between SSv2 and CARLA for this entity is shifted by it -- see
    /// [`OriginOffset`].
    pub origin_offset: OriginOffset,
}

#[derive(Debug, Default)]
pub struct EntityManager {
    entities: HashMap<String, Entity>,
}

impl EntityManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(
        &mut self,
        name: String,
        entity_type: EntityType,
        carla_actor_id: u32,
        origin_offset: OriginOffset,
    ) {
        self.entities.insert(
            name,
            Entity {
                entity_type,
                carla_actor_id,
                origin_offset,
            },
        );
    }

    pub fn remove(&mut self, name: &str) -> Option<u32> {
        self.entities.remove(name).map(|e| e.carla_actor_id)
    }

    pub fn get(&self, name: &str) -> Option<&Entity> {
        self.entities.get(name)
    }

    /// The SSv2 name of the entity backed by a CARLA actor, if any. A linear scan: a
    /// scenario has a handful of entities, and this is only asked when something collides.
    pub fn name_of_actor(&self, carla_actor_id: u32) -> Option<&str> {
        self.entities
            .iter()
            .find(|(_, e)| e.carla_actor_id == carla_actor_id)
            .map(|(name, _)| name.as_str())
    }

    pub fn clear(&mut self) {
        self.entities.clear();
    }

    /// The name of the ego entity, if one is registered.
    pub fn ego_name(&self) -> Option<&str> {
        self.entities
            .iter()
            .find(|(_, e)| e.entity_type == EntityType::Ego)
            .map(|(name, _)| name.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ego_is_found_by_type_not_by_name() {
        let mut m = EntityManager::new();
        assert_eq!(m.ego_name(), None);
        m.insert(
            "npc".into(),
            EntityType::Vehicle,
            1,
            OriginOffset::default(),
        );
        m.insert(
            "walker".into(),
            EntityType::Pedestrian,
            2,
            OriginOffset::default(),
        );
        assert_eq!(m.ego_name(), None);
        m.insert("Ego".into(), EntityType::Ego, 3, OriginOffset::default());
        assert_eq!(m.ego_name(), Some("Ego"));
        m.remove("Ego");
        assert_eq!(m.ego_name(), None);
    }
}
