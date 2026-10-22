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
    /// Actors despawned since the last [`clear`](Self::clear), by CARLA id: a contact CARLA
    /// reported on an entity's last frame can reach swb after SSv2 has despawned it, and
    /// must still be named (roadmap 032 N4).
    departed: HashMap<u32, String>,
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
        self.departed.remove(&carla_actor_id);
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
        let id = self.entities.remove(name)?.carla_actor_id;
        self.departed.insert(id, name.to_owned());
        Some(id)
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

    /// [`name_of_actor`](Self::name_of_actor), or the name of an entity this episode
    /// despawned: for naming contacts, which can be drained after their entity is gone.
    pub fn name_of_actor_this_episode(&self, carla_actor_id: u32) -> Option<&str> {
        self.name_of_actor(carla_actor_id)
            .or_else(|| self.departed.get(&carla_actor_id).map(String::as_str))
    }

    pub fn count(&self) -> usize {
        self.entities.len()
    }

    /// Every registered entity's name.
    pub fn names(&self) -> impl Iterator<Item = String> + '_ {
        self.entities.keys().cloned()
    }

    pub fn clear(&mut self) {
        self.entities.clear();
        self.departed.clear();
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

    #[test]
    fn a_despawned_entity_keeps_its_name_for_contacts_until_clear() {
        let mut m = EntityManager::new();
        m.insert("ego".into(), EntityType::Ego, 1, OriginOffset::default());
        m.insert(
            "npc_rear".into(),
            EntityType::Vehicle,
            2,
            OriginOffset::default(),
        );
        assert_eq!(m.remove("npc_rear"), Some(2));
        // Gone for everything else...
        assert_eq!(m.name_of_actor(2), None);
        assert!(m.get("npc_rear").is_none());
        assert_eq!(m.count(), 1);
        // ...but a late contact with actor 2 is still npc_rear's.
        assert_eq!(m.name_of_actor_this_episode(2), Some("npc_rear"));
        assert_eq!(m.name_of_actor_this_episode(1), Some("ego"));
        assert_eq!(m.name_of_actor_this_episode(3), None);
        // A live entity on the same id wins over the departed one.
        m.insert(
            "other".into(),
            EntityType::Vehicle,
            2,
            OriginOffset::default(),
        );
        assert_eq!(m.name_of_actor_this_episode(2), Some("other"));
        m.remove("other");
        // A new episode forgets them.
        m.clear();
        assert_eq!(m.name_of_actor_this_episode(2), None);
        assert_eq!(m.name_of_actor_this_episode(1), None);
    }
}
