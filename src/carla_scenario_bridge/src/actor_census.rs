//! What actors CARLA holds now, looked up by id (roadmap 034 step 5).
//!
//! `World::GetActors` lists the actors of the last episode state the client received, so a
//! long-lived client misses actors created since its last tick and lists actors destroyed
//! since (the phantom "Destroyed orphaned" lines), and a fresh client in synchronous mode
//! with nobody ticking sees none at all (`Episode.cpp:129-131`). `get_actors_by_id` asks the
//! server (`CarlaServer.cpp:708-723`) -- but LibCarla answers it from the client's
//! `CachedActorList` first, which is never evicted (`CachedActorList.h`; its only `Clear` is
//! in the uncalled `OnEpisodeStarted`), so a destroyed actor the client once saw is still
//! "found". The census therefore asks through a short-lived client of its own, whose cache
//! is empty, over every id from 1 to a margin above the highest id known (ids only
//! increase, `ActorRegistry.cpp:24,85-94`). It reads descriptions only -- type, role,
//! parent -- which need no tick; a pose does.

/// One actor as the census sees it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Found {
    pub id: u32,
    pub type_id: String,
    pub role: Option<String>,
    /// The actor it is attached to, if any.
    pub parent: Option<u32>,
}

/// Ids asked for per lookup.
pub const BLOCK: u32 = 8192;
/// How far above the highest id known the census looks, and above the highest it finds.
pub const MARGIN: u32 = 16384;

/// Every actor with an id from 1 to `MARGIN` above both `watermark` and the highest found,
/// asked `BLOCK` ids at a time through `lookup`.
pub fn scan(
    watermark: u32,
    mut lookup: impl FnMut(&[u32]) -> Result<Vec<Found>, String>,
) -> Result<Vec<Found>, String> {
    let mut found: Vec<Found> = Vec::new();
    let mut next = 1u32;
    let mut top = watermark.saturating_add(MARGIN);
    while next <= top {
        let end = next.saturating_add(BLOCK - 1).min(top);
        let ids: Vec<u32> = (next..=end).collect();
        let block = lookup(&ids)?;
        if let Some(highest) = block.iter().map(|f| f.id).max() {
            top = top.max(highest.saturating_add(MARGIN));
        }
        found.extend(block);
        if end == u32::MAX {
            break;
        }
        next = end + 1;
    }
    found.sort_by_key(|f| f.id);
    found.dedup_by_key(|f| f.id);
    Ok(found)
}

/// The actors a previous bridge process left: entities with csb's mark (`mark_prefix`),
/// and vehicles with a role csb owns (the ego's, the agent ledger's), except this
/// process's own. As `(id, role)`.
pub fn orphans(
    found: &[Found],
    mark_prefix: &str,
    owned_roles: &[String],
    own: impl Fn(u32) -> bool,
) -> Vec<(u32, String)> {
    found
        .iter()
        .filter(|f| !own(f.id))
        .filter_map(|f| {
            let role = f.role.as_ref()?;
            let ours = role.starts_with(mark_prefix)
                || (f.type_id.starts_with("vehicle.") && owned_roles.contains(role));
            ours.then(|| (f.id, role.clone()))
        })
        .collect()
}

/// Sensors whose parent is not in the census: attached to a vehicle that is gone, or to
/// nothing.
pub fn parentless_sensors(found: &[Found]) -> Vec<u32> {
    found
        .iter()
        .filter(|f| f.type_id.starts_with("sensor."))
        .filter(|f| match f.parent {
            None => true,
            Some(p) => !found.iter().any(|g| g.id == p),
        })
        .map(|f| f.id)
        .collect()
}

/// The actors attached to `id`.
pub fn children_of(found: &[Found], id: u32) -> Vec<u32> {
    found
        .iter()
        .filter(|f| f.parent == Some(id))
        .map(|f| f.id)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn actor(id: u32, type_id: &str, role: Option<&str>, parent: Option<u32>) -> Found {
        Found {
            id,
            type_id: type_id.into(),
            role: role.map(str::to_owned),
            parent,
        }
    }

    /// A server holding `world`, answering lookups by id, counting them.
    fn server(world: Vec<Found>) -> impl FnMut(&[u32]) -> Result<Vec<Found>, String> {
        move |ids| {
            Ok(world
                .iter()
                .filter(|f| ids.contains(&f.id))
                .cloned()
                .collect())
        }
    }

    #[test]
    fn the_census_sees_actors_created_after_the_watermark() {
        // The client's last frame knew up to 9003; 9004 (a timed-out spawn's hero) came
        // later, 9010 later still, and both are within the margin.
        let world = vec![
            actor(12, "traffic.traffic_light", None, None),
            actor(9003, "vehicle.tesla.model3", Some("csb_entity:npc"), None),
            actor(9004, "vehicle.lincoln.mkz", Some("hero"), None),
            actor(9010, "sensor.other.imu", Some(""), Some(9004)),
        ];
        let found = scan(9003, server(world.clone())).unwrap();
        assert_eq!(found, world);
    }

    #[test]
    fn the_census_follows_ids_past_its_margin() {
        let far = 9003 + MARGIN + 5000;
        let world = vec![
            actor(9003 + MARGIN - 1, "vehicle.a", Some("hero"), None),
            actor(far, "vehicle.b", Some("hero"), None),
        ];
        let found = scan(9003, server(world)).unwrap();
        assert_eq!(
            found.iter().map(|f| f.id).collect::<Vec<_>>(),
            vec![9003 + MARGIN - 1, far]
        );
    }

    #[test]
    fn the_census_asks_in_blocks_and_stops_above_the_margin() {
        let mut asked = Vec::new();
        let found = scan(100, |ids| {
            asked.push((ids[0], *ids.last().unwrap()));
            Ok(Vec::new())
        })
        .unwrap();
        assert!(found.is_empty());
        assert_eq!(asked.first(), Some(&(1, BLOCK)));
        assert_eq!(asked.last().unwrap().1, 100 + MARGIN);
        assert!(asked.iter().all(|(a, b)| b - a < BLOCK));
    }

    #[test]
    fn a_failed_lookup_fails_the_census() {
        assert!(scan(10, |_| Err("timeout".into())).is_err());
    }

    #[test]
    fn orphans_are_marked_entities_and_owned_vehicles_of_others() {
        let found = vec![
            actor(1, "vehicle.a", Some("hero"), None),
            actor(2, "walker.pedestrian.0001", Some("csb_entity:ped"), None),
            actor(3, "vehicle.b", Some("someone_else"), None),
            actor(4, "sensor.camera.rgb", Some("hero"), Some(1)),
            actor(5, "vehicle.c", Some("csb_entity:mine"), None),
            actor(6, "vehicle.d", Some("bg_av_1"), None),
            actor(7, "static.prop.box", None, None),
        ];
        let owned = vec!["hero".to_string(), "bg_av_1".to_string()];
        let got = orphans(&found, "csb_entity:", &owned, |id| id == 5);
        assert_eq!(
            got,
            vec![
                (1, "hero".to_string()),
                (2, "csb_entity:ped".to_string()),
                (6, "bg_av_1".to_string()),
            ]
        );
    }

    #[test]
    fn a_sensor_is_parentless_when_its_parent_is_not_in_the_census() {
        let found = vec![
            actor(1, "vehicle.a", Some("hero"), None),
            actor(2, "sensor.lidar.ray_cast", None, Some(1)),
            actor(3, "sensor.other.imu", None, Some(99)),
            actor(4, "sensor.camera.rgb", None, None),
        ];
        assert_eq!(parentless_sensors(&found), vec![3, 4]);
        assert_eq!(children_of(&found, 1), vec![2]);
    }
}
