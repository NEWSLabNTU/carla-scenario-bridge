//! CARLA's view of the ego's collisions, logged next to SSv2's (roadmap 014, gap 3).
//!
//! # Why
//!
//! Collision is computed twice and nothing compares the two. SSv2 judges it from its
//! declared bounding boxes and canonical poses; CARLA resolves the PhysX ego against the
//! kinematic NPC colliders (`set_simulate_physics(false)` keeps colliders active). A
//! teleported NPC can shove the ego without SSv2 ever registering a collision, and SSv2 can
//! report one CARLA never rendered. See `docs/design/ssv2-feature-completeness.md` §2.
//!
//! # Policy: diagnostic only
//!
//! **Nothing here changes a verdict.** SSv2 stays the judge -- there is no protocol slot to
//! hand it a CARLA collision anyway -- and no request ever fails because of what this module
//! saw. It attaches a `sensor.other.collision` to the ego, logs each contact at WARN (rate
//! limited per other actor), and at the end of the run logs a table at INFO.
//!
//! # The end-of-run summary is the mismatch report, by hand
//!
//! The roadmap asks for "CARLA collision events vs SSv2 CollisionCondition outcomes". The
//! bridge cannot produce the right-hand column: SSv2 evaluates `CollisionCondition` inside
//! its interpreter and never tells the simulator the outcome. So the summary is the CARLA
//! half, keyed by SSv2 entity name and SSv2 frame, and **a human compares it against the
//! scenario's JUnit result**. A collision in this table on a run SSv2 passed -- or an empty
//! table on a run SSv2 failed on `CollisionCondition` -- is the disagreement gap 3 is about.
//!
//! # Threading
//!
//! CARLA delivers sensor data on its own thread. The callback does as little as it can:
//! it stamps the event with the current SSv2 frame (an atomic the coordinator advances on
//! every `UpdateFrame`) and queues it. The coordinator drains the queue on its own thread,
//! where the entity manager can name the other actor, and all logging happens there.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use carla::client::{Actor, ActorBase, Sensor, World};
use carla::geom::Transform;
use carla::sensor::data::CollisionEvent;
use carla::sensor::SensorDataBase;
use eyre::{eyre, Result};

/// At most one WARN per other actor per this much simulation time. A sustained contact --
/// an NPC teleported into the ego and held there -- reports every physics tick, and the log
/// needs to say that it is happening, not how many ticks it lasted. The count is kept in
/// full for the summary.
pub const WARN_INTERVAL_S: f64 = 1.0;

/// One contact, as the sensor callback saw it. Plain data so it can cross threads and be
/// constructed in tests.
#[derive(Debug, Clone, PartialEq)]
pub struct RawCollision {
    /// SSv2 frame: how many `UpdateFrame`s this bridge had received since `Initialize`
    /// when the event arrived.
    pub ssv2_frame: u64,
    /// CARLA's own frame number for the event.
    pub carla_frame: u64,
    /// CARLA simulation time of the event, in seconds.
    pub sim_time: f64,
    /// The other actor's CARLA id, or `None` for a static part of the world.
    pub other_id: Option<u32>,
    /// The other actor's blueprint id (`vehicle.tesla.model3`, `walker...`), or empty.
    pub other_type: String,
    /// Magnitude of the normal impulse, N·s.
    pub impulse: f64,
}

/// Everything recorded about one other actor over a run.
#[derive(Debug, Clone, PartialEq)]
struct Contact {
    /// SSv2 entity name, when the entity manager knew the actor. Filled in on the first
    /// event where it was known.
    name: Option<String>,
    other_type: String,
    first_ssv2_frame: u64,
    first_sim_time: f64,
    count: u64,
    max_impulse: f64,
    /// Sim time of the last WARN for this actor.
    last_warned_at: Option<f64>,
    /// Events since that WARN that were not logged.
    suppressed: u64,
}

/// The pure half of the monitor: per-actor bookkeeping, the WARN rate limit, and the
/// summary. No CARLA types, so it is unit tested directly.
#[derive(Debug, Default)]
pub struct CollisionLog {
    /// Keyed by the other actor's id; `None` is "static world". A BTreeMap so the summary
    /// is stable for equal first frames.
    contacts: BTreeMap<Option<u32>, Contact>,
}

impl CollisionLog {
    /// Record one event. Returns the WARN line to log, or `None` when this actor was
    /// already reported within the last [`WARN_INTERVAL_S`] of simulation time.
    ///
    /// `name` is the SSv2 entity name of the other actor if it is known right now.
    pub fn record(&mut self, event: &RawCollision, name: Option<&str>) -> Option<String> {
        let contact = self
            .contacts
            .entry(event.other_id)
            .or_insert_with(|| Contact {
                name: None,
                other_type: event.other_type.clone(),
                first_ssv2_frame: event.ssv2_frame,
                first_sim_time: event.sim_time,
                count: 0,
                max_impulse: 0.0,
                last_warned_at: None,
                suppressed: 0,
            });
        if contact.name.is_none() {
            contact.name = name.map(str::to_owned);
        }
        contact.count += 1;
        if event.impulse > contact.max_impulse {
            contact.max_impulse = event.impulse;
        }

        let due = match contact.last_warned_at {
            None => true,
            // Time running backwards means a new timeline (a reload); report afresh.
            Some(last) => event.sim_time - last >= WARN_INTERVAL_S || event.sim_time < last,
        };
        if !due {
            contact.suppressed += 1;
            return None;
        }

        let suppressed = std::mem::take(&mut contact.suppressed);
        contact.last_warned_at = Some(event.sim_time);
        let tail = if suppressed > 0 {
            format!(" (+{suppressed} more contact(s) since the last report)")
        } else {
            String::new()
        };
        Some(format!(
            "CARLA collision: ego hit {} at SSv2 frame {} (CARLA frame {}, t={:.2}s), \
             normal impulse {:.1} N·s{tail}. Diagnostic only; SSv2's CollisionCondition \
             is the verdict.",
            describe(event.other_id, contact.name.as_deref(), &contact.other_type),
            event.ssv2_frame,
            event.carla_frame,
            event.sim_time,
            event.impulse,
        ))
    }

    #[cfg(test)]
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
    }

    /// The end-of-run table, one row per other actor in order of first contact.
    ///
    /// This is the CARLA half of the gap 3 comparison. SSv2 does not report its
    /// `CollisionCondition` outcome to the simulator, so compare this against the
    /// scenario's JUnit result by hand.
    pub fn summary(&self, ego: &str) -> String {
        if self.contacts.is_empty() {
            return format!(
                "CARLA collision summary for ego '{ego}': no collisions. If SSv2 reported a \
                 CollisionCondition for this run, CARLA and SSv2 disagree."
            );
        }

        let mut rows: Vec<(&Option<u32>, &Contact)> = self.contacts.iter().collect();
        rows.sort_by_key(|(_, c)| c.first_ssv2_frame);

        let mut out = format!(
            "CARLA collision summary for ego '{ego}': {} other actor(s). Diagnostic only -- \
             compare against the scenario's JUnit result (SSv2's CollisionCondition is the \
             verdict and is not visible to the bridge).\n",
            rows.len()
        );
        out.push_str(&format!(
            "  {:<32} {:>11} {:>9} {:>6} {:>12}\n",
            "other entity", "first frame", "first t", "count", "max impulse"
        ));
        for (id, c) in rows {
            out.push_str(&format!(
                "  {:<32} {:>11} {:>8.2}s {:>6} {:>8.1} N·s\n",
                describe(*id, c.name.as_deref(), &c.other_type),
                c.first_ssv2_frame,
                c.first_sim_time,
                c.count,
                c.max_impulse
            ));
        }
        out.pop(); // trailing newline
        out
    }
}

/// How an other actor is named in logs: SSv2 name when known, else the CARLA type and id.
fn describe(id: Option<u32>, name: Option<&str>, other_type: &str) -> String {
    let kind = if other_type.is_empty() {
        "?"
    } else {
        other_type
    };
    match (id, name) {
        (None, _) => "static world".to_string(),
        (Some(id), Some(name)) => format!("'{name}' (actor {id}, {kind})"),
        (Some(id), None) => format!("unknown entity (actor {id}, {kind})"),
    }
}

/// The monitor as the coordinator holds it: the sensor, the frame counter the callback
/// reads, and the queue between the callback and the coordinator's thread.
pub struct CollisionMonitor {
    enabled: bool,
    /// SSv2 `UpdateFrame` count since `Initialize`. Shared with the sensor callback.
    frame: Arc<AtomicU64>,
    queue: Arc<Mutex<Vec<RawCollision>>>,
    log: CollisionLog,
    /// The live sensor and the SSv2 name of the ego it is attached to.
    attached: Option<(Sensor, String)>,
}

impl CollisionMonitor {
    pub fn new(enabled: bool) -> Self {
        if !enabled {
            tracing::info!("Collision monitor disabled by config (collision_monitor: false)");
        }
        Self {
            enabled,
            frame: Arc::new(AtomicU64::new(0)),
            queue: Arc::new(Mutex::new(Vec::new())),
            log: CollisionLog::default(),
            attached: None,
        }
    }

    /// Advance the SSv2 frame counter. Call once per `UpdateFrame`, before ticking, so
    /// events from that tick carry that frame.
    pub fn next_frame(&self) -> u64 {
        self.frame.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// Restart frame numbering for a new scenario.
    pub fn reset_frames(&self) {
        self.frame.store(0, Ordering::Relaxed);
    }

    /// Attach a collision sensor to the ego and start listening.
    ///
    /// Best effort: a failure is logged and the run continues without CARLA's view.
    pub fn attach(&mut self, world: &mut World, ego: &Actor, ego_name: &str) {
        if !self.enabled {
            return;
        }
        if self.attached.is_some() {
            // A second ego in one run: close the first one's account before the next.
            self.finish_run(|_| None);
        }
        match self.try_attach(world, ego) {
            Ok(sensor) => {
                tracing::info!(
                    "Collision monitor: sensor.other.collision (actor {}) attached to ego \
                     '{ego_name}' (actor {}); diagnostic only",
                    sensor.id(),
                    ego.id()
                );
                self.attached = Some((sensor, ego_name.to_string()));
            }
            Err(e) => tracing::warn!(
                "Collision monitor: could not attach to ego '{ego_name}' (actor {}): {e:#}. \
                 CARLA collisions will not be logged this run.",
                ego.id()
            ),
        }
    }

    fn try_attach(&self, world: &mut World, ego: &Actor) -> Result<Sensor> {
        let actor = world
            .actor_builder("sensor.other.collision")
            .map_err(|e| eyre!("find blueprint: {e}"))?
            .spawn_opt(
                Transform::from_na(&nalgebra::Isometry3::identity()),
                Some(ego),
                carla::rpc::AttachmentType::Rigid,
            )
            .map_err(|e| eyre!("spawn: {e}"))?;
        let sensor = match Sensor::try_from(actor) {
            Ok(s) => s,
            Err(actor) => {
                // Not a sensor after all; do not leave it attached to the ego.
                let _ = actor.destroy();
                return Err(eyre!("spawned actor is not a sensor"));
            }
        };

        let frame = self.frame.clone();
        let queue = self.queue.clone();
        let listened = sensor.listen(move |data| {
            let carla_frame = data.frame() as u64;
            let sim_time = data.timestamp();
            let Ok(event) = CollisionEvent::try_from(data) else {
                return;
            };
            let v = event.normal_impulse();
            let impulse = ((v.x * v.x + v.y * v.y + v.z * v.z) as f64).sqrt();
            let other = event.other_actor().ok().flatten();
            let raw = RawCollision {
                ssv2_frame: frame.load(Ordering::Relaxed),
                carla_frame,
                sim_time,
                other_id: other.as_ref().map(|a| a.id()),
                other_type: other.as_ref().map(|a| a.type_id()).unwrap_or_default(),
                impulse,
            };
            if let Ok(mut q) = queue.lock() {
                q.push(raw);
            }
        });
        if let Err(e) = listened {
            let _ = sensor.destroy();
            return Err(eyre!("listen: {e}"));
        }
        Ok(sensor)
    }

    /// Log whatever the callback has queued since the last drain. `name_of` maps a CARLA
    /// actor id to its SSv2 entity name.
    pub fn drain<F>(&mut self, mut name_of: F)
    where
        F: FnMut(u32) -> Option<String>,
    {
        let events = match self.queue.lock() {
            Ok(mut q) => std::mem::take(&mut *q),
            Err(_) => return,
        };
        for event in &events {
            let name = event.other_id.and_then(&mut name_of);
            if let Some(line) = self.log.record(event, name.as_deref()) {
                tracing::warn!("{line}");
            }
        }
    }

    /// Drop the sensor without a word to CARLA: the server it lived on is gone (CARLA
    /// restarted), so stopping or destroying it would only wait on a dead connection.
    /// What it recorded is still logged.
    pub fn abandon(&mut self) {
        if let Some((sensor, ego_name)) = self.attached.take() {
            // Leaked, not dropped: LibCarla's ServerSideSensor destructor stops a listening
            // sensor, which is an RPC to the dead server (and an exception thrown from a
            // destructor aborts). One handle per CARLA restart.
            std::mem::forget(sensor);
            self.drain(|_| None);
            tracing::info!(
                "{} (sensor abandoned: CARLA restarted)",
                self.log.summary(&ego_name)
            );
            self.log = CollisionLog::default();
        }
    }

    /// End the run: stop and destroy the sensor, then log the summary.
    ///
    /// Must be called **before** the ego is destroyed. This client is the sensor's
    /// listener, so it stops the stream itself -- destroying a sensor that is still
    /// listened to is what floods CARLA 0.9.16 with `Invalid session` and ends in a server
    /// segfault (acb `docs/issues/015`). If this fails, the ego's `destroy_with_children`
    /// still takes the sensor with it, and `reap_parentless_sensors` catches anything left.
    ///
    /// A no-op when nothing is attached, so it is safe on every teardown path.
    pub fn finish_run<F>(&mut self, name_of: F)
    where
        F: FnMut(u32) -> Option<String>,
    {
        let Some((sensor, ego_name)) = self.attached.take() else {
            return;
        };
        let sensor_id = sensor.id();
        if let Err(e) = sensor.stop() {
            tracing::debug!("Collision sensor {sensor_id} stop failed (already gone?): {e}");
        }
        match sensor.destroy() {
            Ok(_) => tracing::debug!("Collision sensor {sensor_id} destroyed"),
            Err(e) => tracing::debug!(
                "Collision sensor {sensor_id} destroy failed ({e}); the ego's \
                 destroy_with_children will take it"
            ),
        }

        // Anything the callback queued before the stop.
        self.drain(name_of);
        tracing::info!("{}", self.log.summary(&ego_name));
        self.log = CollisionLog::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hit(frame: u64, t: f64, other: Option<u32>, impulse: f64) -> RawCollision {
        RawCollision {
            ssv2_frame: frame,
            carla_frame: frame * 2,
            sim_time: t,
            other_id: other,
            other_type: "vehicle.tesla.model3".into(),
            impulse,
        }
    }

    #[test]
    fn the_first_contact_with_an_actor_warns() {
        let mut log = CollisionLog::default();
        let line = log.record(&hit(42, 4.2, Some(7), 1234.5), Some("npc1"));
        let line = line.expect("first contact must warn");
        assert!(line.contains("'npc1'"), "{line}");
        assert!(line.contains("SSv2 frame 42"), "{line}");
        assert!(line.contains("1234.5"), "{line}");
    }

    /// A sustained contact reports every physics tick; the log says so once a second.
    #[test]
    fn a_sustained_contact_warns_once_per_interval() {
        let mut log = CollisionLog::default();
        let warned: Vec<bool> = (0..25)
            .map(|i| {
                let t = 10.0 + i as f64 * 0.1;
                log.record(&hit(100 + i, t, Some(7), 10.0), Some("npc1"))
                    .is_some()
            })
            .collect();
        // t = 10.0, 11.0, 12.0 (floating point: 10 + 10*0.1 may land a hair under 11.0,
        // so allow the neighbouring tick).
        let count = warned.iter().filter(|w| **w).count();
        assert_eq!(count, 3, "{warned:?}");
        assert!(warned[0]);
    }

    #[test]
    fn the_next_warning_counts_what_was_suppressed() {
        let mut log = CollisionLog::default();
        log.record(&hit(1, 0.0, Some(7), 1.0), None);
        assert!(log.record(&hit(2, 0.1, Some(7), 1.0), None).is_none());
        assert!(log.record(&hit(3, 0.2, Some(7), 1.0), None).is_none());
        let line = log.record(&hit(20, 2.0, Some(7), 1.0), None).unwrap();
        assert!(line.contains("+2 more"), "{line}");
    }

    /// The rate limit is per other actor: a second NPC is news even mid-contact.
    #[test]
    fn different_actors_are_limited_separately() {
        let mut log = CollisionLog::default();
        assert!(log.record(&hit(1, 0.0, Some(7), 1.0), None).is_some());
        assert!(log.record(&hit(1, 0.0, Some(8), 1.0), None).is_some());
        assert!(log.record(&hit(1, 0.0, None, 1.0), None).is_some());
    }

    #[test]
    fn time_running_backwards_reports_afresh() {
        let mut log = CollisionLog::default();
        log.record(&hit(1, 50.0, Some(7), 1.0), None);
        assert!(log.record(&hit(2, 0.1, Some(7), 1.0), None).is_some());
    }

    /// Frame bookkeeping: first frame sticks, count and max impulse accumulate, and a name
    /// learned later fills in.
    #[test]
    fn the_summary_keeps_first_frame_count_and_max_impulse() {
        let mut log = CollisionLog::default();
        log.record(&hit(30, 3.0, Some(7), 5.0), None);
        log.record(&hit(31, 3.1, Some(7), 900.0), Some("cut_in"));
        log.record(&hit(35, 3.5, Some(7), 12.0), Some("cut_in"));
        log.record(&hit(12, 1.2, Some(9), 3.0), Some("ped"));

        let s = log.summary("ego");
        assert!(s.contains("2 other actor(s)"), "{s}");
        let rows: Vec<&str> = s.lines().skip(2).collect();
        assert_eq!(rows.len(), 2, "{s}");
        // Ordered by first contact: the pedestrian (frame 12) before the cut-in (frame 30).
        assert!(rows[0].contains("'ped'"), "{s}");
        assert!(rows[1].contains("'cut_in'"), "{s}");
        let cols: Vec<&str> = rows[1].split_whitespace().collect();
        // ... 'cut_in' (actor 7, vehicle.tesla.model3) | 30 | 3.00s | 3 | 900.0 | N·s
        assert_eq!(
            &cols[cols.len() - 5..],
            ["30", "3.00s", "3", "900.0", "N·s"]
        );
    }

    #[test]
    fn an_empty_run_says_so_and_names_the_comparison() {
        let log = CollisionLog::default();
        assert!(log.is_empty());
        let s = log.summary("ego");
        assert!(s.contains("no collisions"), "{s}");
        assert!(s.contains("CollisionCondition"), "{s}");
    }

    #[test]
    fn unnamed_and_static_contacts_are_described() {
        assert_eq!(describe(None, None, ""), "static world");
        assert_eq!(
            describe(Some(3), None, "walker.pedestrian.0001"),
            "unknown entity (actor 3, walker.pedestrian.0001)"
        );
        assert_eq!(describe(Some(3), Some("p"), ""), "'p' (actor 3, ?)");
    }

    /// The counter the callback stamps events with: one per UpdateFrame, reset per run.
    #[test]
    fn frames_count_update_frames_and_reset() {
        let m = CollisionMonitor::new(false);
        assert_eq!(m.next_frame(), 1);
        assert_eq!(m.next_frame(), 2);
        assert_eq!(m.frame.load(Ordering::Relaxed), 2);
        m.reset_frames();
        assert_eq!(m.next_frame(), 1);
    }

    /// Draining routes queued events through the log with names from the caller.
    #[test]
    fn drain_names_events_and_empties_the_queue() {
        let mut m = CollisionMonitor::new(false);
        m.queue.lock().unwrap().push(hit(5, 0.5, Some(7), 2.0));
        m.queue.lock().unwrap().push(hit(6, 0.6, Some(7), 4.0));
        m.drain(|id| (id == 7).then(|| "npc1".to_string()));
        assert!(m.queue.lock().unwrap().is_empty());
        let s = m.log.summary("ego");
        assert!(s.contains("'npc1'"), "{s}");
        assert!(s.contains("4.0 N·s"), "{s}");
    }
}
