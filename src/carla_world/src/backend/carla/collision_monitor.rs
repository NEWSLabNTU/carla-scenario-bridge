//! CARLA's view of the ego's collisions, logged next to SSv2's (roadmap 014, gap 3).
//!
//! # Why
//!
//! Collision is computed twice and nothing compares the two. SSv2 judges it from its
//! declared bounding boxes and canonical poses; CARLA resolves the PhysX ego against the
//! NPCs' bodies (tracked physics bodies by default, roadmap 023; kinematic colliders under
//! `npc_motion: kinematic`, where `set_simulate_physics(false)` keeps colliders active). An
//! NPC can shove the ego without SSv2 ever registering a collision, and SSv2 can report one
//! CARLA never rendered. See `docs/design/ssv2-feature-completeness.md` §2.
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
//! # NPC sensors (roadmap 032 step 5)
//!
//! With `npc_contact_sensors` on (the default), every tracked NPC vehicle carries a sensor of
//! its own, so an NPC's contacts with the static world and with other NPCs -- invisible to
//! the ego's sensor -- reach the gap trace (`subject` on the contact line) and the oracle's
//! low-priority G2. Their events are kept apart from the ego's everywhere the ego is judged:
//! not in the ego's log or its report's `contacts` (they go to `npc_contacts`), and not in
//! the ego-contact window the tracker holds an NPC by (`touched_within`). Each NPC sensor is
//! stopped and destroyed before its vehicle ([`CollisionMonitor::detach_npc`]), on every path
//! that destroys one.
//!
//! # Threading
//!
//! CARLA delivers sensor data on its own thread. The callback does as little as it can:
//! it stamps the event with the current SSv2 frame (an atomic the coordinator advances on
//! every `UpdateFrame`) and queues it. The coordinator drains the queue on its own thread,
//! where the entity manager can name the other actor, and all logging happens there.
//!
//! # Labels: the CARLA frame, not the arrival (roadmap 032 N2)
//!
//! The stream can deliver an event frames after its tick: in `rear_contact` run 3 the
//! ego-`npc_rear` impact of SSv2 frame 265 reached the callback at 269, and the blocked rule,
//! which then saw no ego contact, handed `npc_rear` to kinematic into the ego. So the
//! coordinator reports every tick's returned CARLA frame ([`CollisionMonitor::note_tick`]),
//! and a drained event is labelled with the SSv2 frame whose tick produced its CARLA frame --
//! in the log, the report, the gap trace and the contact window alike. The callback's stamp
//! is only the fallback for a CARLA frame no tick of this episode returned.

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use carla::client::{Actor, ActorBase, Sensor, World};
use carla::geom::Transform;
use carla::sensor::data::CollisionEvent;
use carla::sensor::SensorDataBase;
use eyre::{eyre, Result};

pub use crate::contact_report::RawCollision;
use crate::contact_report::{
    self, ContactReport, ReportContact, ReportGeometry, ReportNpcContact, ReportTracking,
};
use crate::npc_tracking::TrackingStats;

/// At most one WARN per other actor per this much simulation time. A sustained contact --
/// an NPC teleported into the ego and held there -- reports every physics tick, and the log
/// needs to say that it is happening, not how many ticks it lasted. The count is kept in
/// full for the summary.
pub const WARN_INTERVAL_S: f64 = 1.0;

/// Ticks remembered for labelling events by CARLA frame: 4096 ticks are 200 s at 20 Hz with
/// one tick per frame (the stream lags by frames, not minutes), about 100 KB.
const TICK_MAP_LEN: usize = 4096;

/// The SSv2 frame of an event at `carla_frame`: that of the tick that returned it, or, for a
/// CARLA frame between two noted ticks, of the later one. `stamped` (the callback's counter)
/// when it lies outside every noted tick.
fn label_frame(ticks: &BTreeMap<u64, u64>, carla_frame: u64, stamped: u64) -> u64 {
    if let Some(&frame) = ticks.get(&carla_frame) {
        return frame;
    }
    let after = ticks.range(carla_frame..).next();
    let before = ticks.range(..carla_frame).next_back();
    match (before, after) {
        (Some(_), Some((_, &frame))) => frame,
        _ => stamped,
    }
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
        let who = match &event.subject {
            None => "ego".to_string(),
            Some(npc) => format!("NPC '{npc}'"),
        };
        Some(format!(
            "CARLA collision: {who} hit {} at SSv2 frame {} (CARLA frame {}, t={:.2}s), \
             normal impulse {:.1} N·s{tail}. Diagnostic only; SSv2's CollisionCondition \
             is the verdict.",
            describe(event.other_id, contact.name.as_deref(), &contact.other_type),
            event.ssv2_frame,
            event.carla_frame,
            event.sim_time,
            event.impulse,
        ))
    }

    /// The run's contacts in the shape of the runner's contact report, in order of first
    /// contact.
    pub fn report_contacts(&self) -> Vec<ReportContact> {
        let mut rows: Vec<ReportContact> = self
            .contacts
            .iter()
            .map(|(id, c)| ReportContact {
                other_id: *id,
                name: c.name.clone(),
                other_type: c.other_type.clone(),
                is_static: id.is_none(),
                first_ssv2_frame: c.first_ssv2_frame,
                first_sim_time: c.first_sim_time,
                count: c.count,
                max_impulse: c.max_impulse,
            })
            .collect();
        rows.sort_by_key(|r| r.first_ssv2_frame);
        rows
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
/// One tracking record of one frame (what [`TrackingStats`] counts).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrackingSample {
    Measured {
        error: f64,
        yaw_error: Option<f64>,
        snapped: bool,
    },
    Missed,
    Fallback,
}

pub struct CollisionMonitor {
    enabled: bool,
    /// SSv2 `UpdateFrame` count since `Initialize`. Shared with the sensor callback.
    frame: Arc<AtomicU64>,
    queue: Arc<Mutex<Vec<RawCollision>>>,
    log: CollisionLog,
    /// The live sensor and the SSv2 name of the ego it is attached to.
    attached: Option<(Sensor, String)>,
    /// Where each run's contact report goes (roadmap 021); `None` writes none.
    report_dir: Option<PathBuf>,
    /// `Initialize` count, for the report.
    episode: u64,
    /// Wall time the sensor was attached: the run's start in the report.
    run_start_unix: f64,
    /// Each entity's body against its declared box, this episode (roadmap 022).
    geometry: Vec<ReportGeometry>,
    /// Each tracked NPC's tracking error, this episode (roadmap 023).
    tracking: std::collections::BTreeMap<String, TrackingStats>,
    /// This frame's tracking records, for the world protocol (roadmap 046), drained by
    /// [`CollisionMonitor::take_tracking_samples`].
    samples: Vec<(String, TrackingSample)>,
    /// The SSv2 frame of each actor's latest contact with the ego, this episode.
    last_contact: std::collections::HashMap<u32, u64>,
    /// Every drained event with the other actor's SSv2 name, kept for the gap trace
    /// (roadmap 024) until [`CollisionMonitor::take_traced`]; `None` keeps nothing.
    traced: Option<Vec<(RawCollision, Option<String>)>>,
    /// Attach a sensor to every tracked NPC vehicle too (`npc_contact_sensors`, roadmap 032).
    npc_sensors_enabled: bool,
    /// The NPC sensors, by the NPC's actor id, with the NPC's SSv2 name.
    npc_sensors: HashMap<u32, (Sensor, String)>,
    /// What each NPC's sensor saw, by NPC name; never mixed into `log` (the ego's).
    npc_logs: BTreeMap<String, CollisionLog>,
    /// CARLA frame returned by each tick -> the SSv2 frame it was ticked in, this episode.
    tick_frames: BTreeMap<u64, u64>,
    /// Events this episode whose label differs from the callback's stamp (delivered late).
    relabelled: u64,
}

impl CollisionMonitor {
    pub fn new(enabled: bool, report_dir: Option<PathBuf>) -> Self {
        if !enabled {
            tracing::info!(
                "Collision monitor disabled by config (collision_monitor: false); no contact \
                 reports either"
            );
        } else {
            match &report_dir {
                Some(dir) => tracing::info!("Contact reports: {}", dir.display()),
                None => tracing::info!("Contact reports off (contact_report_dir: off)"),
            }
        }
        Self {
            enabled,
            frame: Arc::new(AtomicU64::new(0)),
            queue: Arc::new(Mutex::new(Vec::new())),
            log: CollisionLog::default(),
            attached: None,
            report_dir: if enabled { report_dir } else { None },
            episode: 0,
            run_start_unix: 0.0,
            geometry: Vec::new(),
            tracking: Default::default(),
            samples: Vec::new(),
            last_contact: Default::default(),
            traced: None,
            npc_sensors_enabled: false,
            npc_sensors: HashMap::new(),
            npc_logs: BTreeMap::new(),
            tick_frames: BTreeMap::new(),
            relabelled: 0,
        }
    }

    /// A tick returned CARLA frame `carla_frame` during the current SSv2 frame. Call after
    /// every successful `world.tick()`, settling ticks included.
    pub fn note_tick(&mut self, carla_frame: u64) {
        self.tick_frames
            .insert(carla_frame, self.frame.load(Ordering::Relaxed));
        while self.tick_frames.len() > TICK_MAP_LEN {
            self.tick_frames.pop_first();
        }
    }

    /// Give tracked NPC vehicles sensors of their own (`npc_contact_sensors`). Nothing when
    /// the monitor itself is off.
    pub fn set_npc_sensors(&mut self, on: bool) {
        self.npc_sensors_enabled = on && self.enabled;
        tracing::info!(
            "NPC contact sensors {} (npc_contact_sensors{})",
            if self.npc_sensors_enabled {
                "on"
            } else {
                "off"
            },
            if on && !self.enabled {
                ", but the collision monitor is off"
            } else {
                ""
            }
        );
    }

    /// Keep drained events for [`CollisionMonitor::take_traced`] (the gap trace is on).
    pub fn keep_for_trace(&mut self) {
        self.traced.get_or_insert_with(Vec::new);
    }

    /// The events drained since the last call, with the other actor's SSv2 name.
    pub fn take_traced(&mut self) -> Vec<(RawCollision, Option<String>)> {
        self.traced.as_mut().map(std::mem::take).unwrap_or_default()
    }

    /// The current SSv2 frame: `UpdateFrame`s since `Initialize`.
    pub fn frame(&self) -> u64 {
        self.frame.load(Ordering::Relaxed)
    }

    /// `Initialize` count of this bridge process (the contact report's `episode`).
    pub fn episode(&self) -> u64 {
        self.episode
    }

    /// Advance the SSv2 frame counter. Call once per `UpdateFrame`, before ticking, so
    /// events from that tick carry that frame.
    pub fn next_frame(&self) -> u64 {
        self.frame.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// A new scenario: restart frame numbering and count the episode.
    pub fn begin_episode(&mut self) {
        self.frame.store(0, Ordering::Relaxed);
        self.episode += 1;
        self.geometry.clear();
        self.tracking.clear();
        self.last_contact.clear();
        self.npc_logs.clear();
        self.tick_frames.clear();
        self.relabelled = 0;
        if let Some(traced) = self.traced.as_mut() {
            traced.clear();
        }
    }

    /// Whether the ego touched `actor_id` within the last `frames` SSv2 frames (as drained:
    /// call after `drain`).
    pub fn touched_within(&self, actor_id: u32, frames: u64) -> bool {
        let now = self.frame.load(Ordering::Relaxed);
        self.last_contact
            .get(&actor_id)
            .is_some_and(|&f| now.saturating_sub(f) <= frames)
    }

    /// One frame of a tracked NPC: its in-plane error (m), yaw error (degrees; `None` for a
    /// walker), and whether it was snapped back.
    pub fn record_tracking(
        &mut self,
        name: &str,
        error: f64,
        yaw_error: Option<f64>,
        snapped: bool,
    ) {
        self.tracking
            .entry(name.to_string())
            .or_default()
            .record(error, yaw_error, snapped);
        self.samples.push((
            name.to_string(),
            TrackingSample::Measured {
                error,
                yaw_error,
                snapped,
            },
        ));
    }

    /// A tracked NPC skipped this frame: it was not in the world snapshot.
    pub fn record_tracking_missed(&mut self, name: &str) {
        self.tracking
            .entry(name.to_string())
            .or_default()
            .record_missed();
        self.samples
            .push((name.to_string(), TrackingSample::Missed));
    }

    /// A tracked NPC driven kinematic this frame after a blocked fallback (roadmap 032).
    pub fn record_tracking_fallback(&mut self, name: &str) {
        self.tracking
            .entry(name.to_string())
            .or_default()
            .record_fallback();
        self.samples
            .push((name.to_string(), TrackingSample::Fallback));
    }

    /// The tracking records since the last call, in order.
    pub fn take_tracking_samples(&mut self) -> Vec<(String, TrackingSample)> {
        std::mem::take(&mut self.samples)
    }

    fn tracking_rows(&self) -> Vec<ReportTracking> {
        self.tracking
            .iter()
            .map(|(name, stats)| {
                let s = stats.summary();
                ReportTracking {
                    name: name.clone(),
                    frames: s.frames,
                    error_p95: s.error_p95,
                    error_max: s.error_max,
                    yaw_p95: s.yaw_p95,
                    yaw_max: s.yaw_max,
                    snaps: s.snaps,
                    missed: s.missed,
                    fallback: s.fallback,
                }
            })
            .collect()
    }

    /// One log line per tracked NPC; nothing under `kinematic`.
    fn log_tracking(&self) {
        for row in self.tracking_rows() {
            let yaw = match (row.yaw_p95, row.yaw_max) {
                (Some(p), Some(m)) => format!(", yaw p95 {p:.2} / max {m:.2} deg"),
                _ => String::new(),
            };
            tracing::info!(
                "Tracked '{}': {} frames, error p95 {:.3} / max {:.3} m{yaw}, {} snap(s), \
                 {} missed{}",
                row.name,
                row.frames,
                row.error_p95,
                row.error_max,
                row.snaps,
                row.missed,
                if row.fallback > 0 {
                    format!(
                        ", {} frame(s) kinematic after a blocked fallback",
                        row.fallback
                    )
                } else {
                    String::new()
                }
            );
        }
    }

    /// Note a spawned entity's body for the episode's reports. A respawn under the same
    /// name replaces the earlier record.
    pub fn record_geometry(&mut self, record: ReportGeometry) {
        self.geometry.retain(|g| g.name != record.name);
        self.geometry.push(record);
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
        match self.try_attach(world, ego, None) {
            Ok(sensor) => {
                tracing::info!(
                    "Collision monitor: sensor.other.collision (actor {}) attached to ego \
                     '{ego_name}' (actor {}); diagnostic only",
                    sensor.id(),
                    ego.id()
                );
                self.attached = Some((sensor, ego_name.to_string()));
                self.run_start_unix = contact_report::now_unix();
            }
            Err(e) => tracing::warn!(
                "Collision monitor: could not attach to ego '{ego_name}' (actor {}): {e:#}. \
                 CARLA collisions will not be logged this run.",
                ego.id()
            ),
        }
    }

    /// Attach a collision sensor to a tracked NPC vehicle (roadmap 032 step 5), when NPC
    /// sensors are on. Best effort, like the ego's: a failure is logged and the NPC runs
    /// without one. Pair every call with [`CollisionMonitor::detach_npc`] before the NPC is
    /// destroyed.
    pub fn attach_npc(&mut self, world: &mut World, npc: &Actor, name: &str) {
        if !self.npc_sensors_enabled {
            return;
        }
        // A respawn on a reused id: the old sensor first.
        self.detach_npc(npc.id());
        match self.try_attach(world, npc, Some(name.to_string())) {
            Ok(sensor) => {
                tracing::debug!(
                    "Collision sensor (actor {}) attached to NPC '{name}' (actor {})",
                    sensor.id(),
                    npc.id()
                );
                self.npc_sensors
                    .insert(npc.id(), (sensor, name.to_string()));
            }
            Err(e) => tracing::warn!(
                "Could not attach a collision sensor to NPC '{name}' (actor {}): {e:#}. Its \
                 contacts will not be seen.",
                npc.id()
            ),
        }
    }

    /// Stop and destroy an NPC's sensor, if it has one. Must run **before** the NPC is
    /// destroyed, for the reason [`CollisionMonitor::finish_run`] gives for the ego's.
    ///
    /// Returns the sensor's actor id, if one was attached: the despawn that follows tells it
    /// apart from sensors somebody else attached (033 step 5).
    pub fn detach_npc(&mut self, npc_id: u32) -> Option<u32> {
        let (sensor, name) = self.npc_sensors.remove(&npc_id)?;
        let sensor_id = sensor.id();
        stop_and_destroy(sensor, &format!("NPC '{name}'"));
        Some(sensor_id)
    }

    /// [`CollisionMonitor::detach_npc`] for every NPC: before a teardown.
    pub fn detach_all_npcs(&mut self) {
        let ids: Vec<u32> = self.npc_sensors.keys().copied().collect();
        for id in ids {
            self.detach_npc(id);
        }
    }

    /// Drop an NPC's sensor handle without a word to CARLA (unreachable or restarted; see
    /// [`CollisionMonitor::abandon`]).
    pub fn forget_npc(&mut self, npc_id: u32) {
        if let Some((sensor, _)) = self.npc_sensors.remove(&npc_id) {
            std::mem::forget(sensor);
        }
    }

    /// What NPCs' own sensors saw, in the report's shape, in order of first contact.
    pub fn npc_report_contacts(&self) -> Vec<ReportNpcContact> {
        let mut rows: Vec<ReportNpcContact> = self
            .npc_logs
            .iter()
            .flat_map(|(npc, log)| {
                log.report_contacts()
                    .into_iter()
                    .map(move |contact| ReportNpcContact {
                        npc: npc.clone(),
                        contact,
                    })
            })
            .collect();
        rows.sort_by_key(|r| r.contact.first_ssv2_frame);
        rows
    }

    fn try_attach(
        &self,
        world: &mut World,
        parent: &Actor,
        subject: Option<String>,
    ) -> Result<Sensor> {
        let actor = world
            .actor_builder("sensor.other.collision")
            .map_err(|e| eyre!("find blueprint: {e}"))?
            .spawn_opt(
                Transform::from_na(&nalgebra::Isometry3::identity()),
                Some(parent),
                carla::rpc::AttachmentType::Rigid,
            )
            .map_err(|e| eyre!("spawn: {e}"))?;
        let sensor = match Sensor::try_from(actor) {
            Ok(s) => s,
            Err(actor) => {
                // Not a sensor after all; do not leave it attached to its parent.
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
                other_id: other.as_ref().and_then(|a| actor_or_world(a.id())),
                other_type: other.as_ref().map(|a| a.type_id()).unwrap_or_default(),
                impulse,
                subject: subject.clone(),
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
        for mut event in events {
            let label = label_frame(&self.tick_frames, event.carla_frame, event.ssv2_frame);
            if label != event.ssv2_frame {
                self.relabelled += 1;
                tracing::debug!(
                    "Contact at CARLA frame {} delivered in SSv2 frame {}: labelled {label}",
                    event.carla_frame,
                    event.ssv2_frame
                );
                event.ssv2_frame = label;
            }
            let event = &event;
            let name = event.other_id.and_then(&mut name_of);
            match &event.subject {
                None => {
                    if let Some(id) = event.other_id {
                        let last = self.last_contact.entry(id).or_insert(event.ssv2_frame);
                        *last = (*last).max(event.ssv2_frame);
                    }
                    if let Some(line) = self.log.record(event, name.as_deref()) {
                        tracing::warn!("{line}");
                    }
                }
                // An NPC's own sensor: its log, at INFO -- an NPC on a curb is not the ego's
                // news -- and never the ego-contact window.
                Some(npc) => {
                    let log = self.npc_logs.entry(npc.clone()).or_default();
                    if let Some(line) = log.record(event, name.as_deref()) {
                        tracing::info!("{line}");
                    }
                }
            }
            if let Some(traced) = self.traced.as_mut() {
                traced.push((event.clone(), name));
            }
        }
    }

    /// Drop the sensor without a word to CARLA: the server it lived on is gone (CARLA
    /// restarted), so stopping or destroying it would only wait on a dead connection.
    /// What it recorded is still logged.
    pub fn abandon(&mut self) {
        for (_, (sensor, _)) in self.npc_sensors.drain() {
            std::mem::forget(sensor);
        }
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
            self.write_report(&ego_name, false);
            self.log = CollisionLog::default();
            self.npc_logs.clear();
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
        stop_and_destroy(sensor, "the ego");

        // Anything the callback queued before the stop.
        self.drain(name_of);
        tracing::info!("{}", self.log.summary(&ego_name));
        if self.relabelled > 0 {
            tracing::info!(
                "{} contact event(s) this run reached csb after the SSv2 frame they happened \
                 in; labelled by their CARLA frame",
                self.relabelled
            );
        }
        self.log_npc_contacts();
        self.log_tracking();
        self.write_report(&ego_name, true);
        self.log = CollisionLog::default();
        self.npc_logs.clear();
    }

    /// One line per NPC that touched anything, while the ego lived.
    fn log_npc_contacts(&self) {
        for row in self.npc_report_contacts() {
            let c = &row.contact;
            tracing::info!(
                "NPC '{}' contacts: {} from SSv2 frame {}, {} event(s), max impulse {:.1} N·s",
                row.npc,
                describe(c.other_id, c.name.as_deref(), &c.other_type),
                c.first_ssv2_frame,
                c.count,
                c.max_impulse
            );
        }
    }

    /// Hand the run's record to the report writer thread, if reports are on.
    fn write_report(&self, ego_name: &str, complete: bool) {
        let Some(dir) = self.report_dir.clone() else {
            return;
        };
        let report = ContactReport {
            schema: contact_report::SCHEMA,
            episode: self.episode,
            ego: ego_name.to_string(),
            start_unix: self.run_start_unix,
            end_unix: contact_report::now_unix(),
            complete,
            contacts: self.log.report_contacts(),
            geometry: self.geometry.clone(),
            tracking: self.tracking_rows(),
            npc_contacts: self.npc_report_contacts(),
        };
        contact_report::write_in_background(dir, report);
    }
}

/// The other actor's id as the log keys it: `None` for the static world. CARLA 0.9.16 hands
/// the world back as an actor with id 0 (`static.pole`, `static.building`, ...), not as no
/// actor -- measured on the bench's `npc_05` lamp pole (roadmap 032 step 5) -- so without
/// this a world hit reads as an unknown actor 0. Spawned props have real ids.
fn actor_or_world(id: u32) -> Option<u32> {
    (id != 0).then_some(id)
}

/// Stop a sensor this client listens to, then destroy it: in that order, or CARLA 0.9.16
/// floods the stream with `Invalid session` and can segfault (acb `docs/issues/015`). A
/// failure is only logged: its parent's `destroy_with_children` takes whatever is left.
fn stop_and_destroy(sensor: Sensor, on: &str) {
    let sensor_id = sensor.id();
    if let Err(e) = sensor.stop() {
        tracing::debug!("Collision sensor {sensor_id} on {on}: stop failed (already gone?): {e}");
    }
    match sensor.destroy() {
        Ok(_) => tracing::debug!("Collision sensor {sensor_id} on {on} destroyed"),
        Err(e) => tracing::debug!(
            "Collision sensor {sensor_id} on {on}: destroy failed ({e}); its parent's \
             destroy_with_children will take it"
        ),
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
            subject: None,
        }
    }

    /// The same event as an NPC's own sensor saw it (roadmap 032 step 5).
    fn npc_hit(npc: &str, frame: u64, other: Option<u32>, impulse: f64) -> RawCollision {
        RawCollision {
            subject: Some(npc.into()),
            ..hit(frame, 0.05 * frame as f64, other, impulse)
        }
    }

    #[test]
    fn actor_zero_is_the_static_world() {
        assert_eq!(actor_or_world(0), None);
        assert_eq!(actor_or_world(233), Some(233));
    }

    #[test]
    fn an_npc_warning_names_the_npc_not_the_ego() {
        let mut log = CollisionLog::default();
        let line = log
            .record(&npc_hit("npc_05", 25, None, 4100.0), None)
            .unwrap();
        assert!(line.contains("NPC 'npc_05' hit static world"), "{line}");
        assert!(!line.contains("ego hit"), "{line}");
    }

    /// NPC sensors feed neither the ego's report nor the ego-contact window the tracker
    /// holds NPCs by; they get their own list, by NPC.
    #[test]
    fn npc_events_stay_out_of_the_ego_report_and_the_contact_window() {
        let mut m = CollisionMonitor::new(false, None);
        m.keep_for_trace();
        m.frame.store(30, Ordering::Relaxed);
        {
            let mut q = m.queue.lock().unwrap();
            q.push(hit(28, 1.4, Some(7), 50.0)); // ego - npc_rear
            q.push(npc_hit("npc_rear", 28, Some(1), 50.0)); // npc_rear - ego
            q.push(npc_hit("npc_05", 25, None, 4100.0)); // npc_05 - lamp pole
            q.push(npc_hit("npc_05", 26, None, 900.0));
            q.push(npc_hit("npc_05", 29, Some(9), 10.0)); // npc_05 - npc_09
        }
        m.drain(|id| match id {
            1 => Some("ego".into()),
            7 => Some("npc_rear".into()),
            9 => Some("npc_09".into()),
            _ => None,
        });
        let ego = m.log.report_contacts();
        assert_eq!(ego.len(), 1);
        assert_eq!(ego[0].name.as_deref(), Some("npc_rear"));
        assert!(m.touched_within(7, 5));
        // npc_05's contact with npc_09 is not the ego touching npc_09.
        assert!(!m.touched_within(9, 5));
        assert!(!m.touched_within(1, 5));

        let npc = m.npc_report_contacts();
        let rows: Vec<(&str, Option<&str>, bool, u64, u64)> = npc
            .iter()
            .map(|r| {
                (
                    r.npc.as_str(),
                    r.contact.name.as_deref(),
                    r.contact.is_static,
                    r.contact.first_ssv2_frame,
                    r.contact.count,
                )
            })
            .collect();
        assert_eq!(
            rows,
            [
                ("npc_05", None, true, 25, 2),
                ("npc_rear", Some("ego"), false, 28, 1),
                ("npc_05", Some("npc_09"), false, 29, 1),
            ]
        );
        // The trace gets every event, the subject kept.
        let traced = m.take_traced();
        assert_eq!(traced.len(), 5);
        assert_eq!(traced[2].0.subject.as_deref(), Some("npc_05"));
        assert_eq!(traced[0].0.subject, None);
        // A new episode forgets them.
        m.begin_episode();
        assert!(m.npc_report_contacts().is_empty());
    }

    #[test]
    fn the_report_lists_contacts_in_order_of_first_contact() {
        let mut log = CollisionLog::default();
        log.record(&hit(50, 5.0, None, 10.0), None);
        log.record(&hit(42, 4.2, Some(7), 100.0), Some("npc1"));
        log.record(&hit(43, 4.3, Some(7), 300.0), Some("npc1"));
        log.record(&hit(60, 6.0, Some(9), 5.0), None);
        let rows = log.report_contacts();
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].name.as_deref(), Some("npc1"));
        assert_eq!((rows[0].count, rows[0].max_impulse), (2, 300.0));
        assert!(rows[1].is_static && rows[1].other_id.is_none());
        assert_eq!((rows[2].other_id, rows[2].name.as_deref()), (Some(9), None));
        assert!(!rows[2].is_static);
    }

    #[test]
    fn an_empty_run_reports_no_contacts() {
        assert!(CollisionLog::default().report_contacts().is_empty());
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
    fn frames_count_update_frames_and_reset_per_episode() {
        let mut m = CollisionMonitor::new(false, None);
        assert_eq!(m.next_frame(), 1);
        assert_eq!(m.next_frame(), 2);
        assert_eq!(m.frame.load(Ordering::Relaxed), 2);
        m.begin_episode();
        assert_eq!(m.next_frame(), 1);
        assert_eq!(m.episode, 1);
    }

    /// Geometry is per episode, one record per entity name.
    #[test]
    fn geometry_records_replace_by_name_and_reset_per_episode() {
        let mut m = CollisionMonitor::new(false, None);
        let rec = |name: &str, bp: &str| ReportGeometry {
            name: name.into(),
            blueprint: bp.into(),
            declared: [4.5, 2.1, 1.8],
            actual: [4.792, 2.163, 1.488],
            fits: true,
        };
        m.record_geometry(rec("npc", "vehicle.tesla.model3"));
        m.record_geometry(rec("ego", "vehicle.tesla.model3"));
        m.record_geometry(rec("npc", "vehicle.audi.a2"));
        let names: Vec<_> = m
            .geometry
            .iter()
            .map(|g| (g.name.as_str(), g.blueprint.as_str()))
            .collect();
        assert_eq!(
            names,
            [("ego", "vehicle.tesla.model3"), ("npc", "vehicle.audi.a2")]
        );
        m.begin_episode();
        assert!(m.geometry.is_empty());
    }

    #[test]
    fn tracking_accumulates_per_name_and_resets_per_episode() {
        let mut m = CollisionMonitor::new(true, None);
        m.record_tracking("npc", 0.1, Some(1.0), false);
        m.record_tracking("npc", 0.6, Some(6.0), true);
        m.record_tracking("walker", 0.05, None, false);
        m.record_tracking_missed("npc");
        m.record_tracking_fallback("npc");
        let rows = m.tracking_rows();
        assert_eq!(rows.len(), 2);
        let npc = rows.iter().find(|r| r.name == "npc").unwrap();
        assert_eq!(
            (npc.frames, npc.snaps, npc.missed, npc.fallback),
            (2, 1, 1, 1)
        );
        assert!((npc.error_max - 0.6).abs() < 1e-6);
        let walker = rows.iter().find(|r| r.name == "walker").unwrap();
        assert_eq!(walker.yaw_p95, None);
        m.begin_episode();
        assert!(m.tracking_rows().is_empty());
    }

    /// Draining routes queued events through the log with names from the caller.
    #[test]
    fn drain_names_events_and_empties_the_queue() {
        let mut m = CollisionMonitor::new(false, None);
        m.queue.lock().unwrap().push(hit(5, 0.5, Some(7), 2.0));
        m.queue.lock().unwrap().push(hit(6, 0.6, Some(7), 4.0));
        m.drain(|id| (id == 7).then(|| "npc1".to_string()));
        assert!(m.queue.lock().unwrap().is_empty());
        let s = m.log.summary("ego");
        assert!(s.contains("'npc1'"), "{s}");
        assert!(s.contains("4.0 N·s"), "{s}");
    }

    #[test]
    fn a_contact_is_remembered_for_a_window_of_frames() {
        let mut m = CollisionMonitor::new(false, None);
        m.frame.store(10, Ordering::Relaxed);
        m.queue.lock().unwrap().push(hit(10, 0.5, Some(7), 2.0));
        m.drain(|_| None);
        assert!(m.touched_within(7, 20));
        assert!(!m.touched_within(8, 20));
        m.frame.store(30, Ordering::Relaxed);
        assert!(m.touched_within(7, 20));
        m.frame.store(31, Ordering::Relaxed);
        assert!(!m.touched_within(7, 20));
        m.begin_episode();
        assert!(!m.touched_within(7, 1_000));
    }

    /// rear_contact run 3 (roadmap 032 N2): the impact was CARLA frame 21787317, ticked in
    /// SSv2 frame 265, but its callback ran -- and stamped the counter -- at 269. The
    /// contact is SSv2 frame 265 everywhere: the log, the trace, the contact window.
    #[test]
    fn a_late_contact_is_labelled_by_the_carla_frame_it_happened_in() {
        let mut m = CollisionMonitor::new(false, None);
        m.keep_for_trace();
        m.frame.store(264, Ordering::Relaxed);
        // Two ticks per SSv2 frame (substeps), 265 to 269.
        let mut carla = 21_787_315;
        for _ in 265..=269 {
            m.next_frame();
            for _ in 0..2 {
                carla += 1;
                m.note_tick(carla);
            }
        }
        assert_eq!(m.frame(), 269);
        let late = RawCollision {
            ssv2_frame: 269,
            carla_frame: 21_787_317,
            ..hit(0, 376.3, Some(7), 7910.0)
        };
        // 21787317 is 265's second substep; 21787318 is 266's first, 21787320 267's: all
        // three delivered at 269.
        let late2 = RawCollision {
            carla_frame: 21_787_320,
            ..late.clone()
        };
        let mid = RawCollision {
            carla_frame: 21_787_318,
            ..late.clone()
        };
        m.queue.lock().unwrap().extend([late, mid, late2]);
        m.drain(|_| Some("npc_rear".into()));
        let traced: Vec<u64> = m.take_traced().iter().map(|(e, _)| e.ssv2_frame).collect();
        assert_eq!(traced, vec![265, 266, 267]);
        assert_eq!(m.log.report_contacts()[0].first_ssv2_frame, 265);
        // The 20-frame hold counts from 267 (the last contact), not from 269.
        m.frame.store(287, Ordering::Relaxed);
        assert!(m.touched_within(7, 20));
        m.frame.store(288, Ordering::Relaxed);
        assert!(!m.touched_within(7, 20));
    }

    /// A CARLA frame this bridge never saw a tick return for -- older than the map, or from
    /// before the episode -- keeps the callback's stamp.
    #[test]
    fn an_unmapped_carla_frame_keeps_the_callbacks_stamp() {
        let mut m = CollisionMonitor::new(false, None);
        m.keep_for_trace();
        m.next_frame();
        m.note_tick(500);
        m.next_frame();
        m.note_tick(501);
        let before = RawCollision {
            carla_frame: 400,
            ..hit(2, 0.1, Some(7), 1.0)
        };
        let after = RawCollision {
            carla_frame: 900,
            ..hit(2, 0.1, Some(7), 1.0)
        };
        m.queue.lock().unwrap().extend([before, after]);
        m.drain(|_| None);
        let traced: Vec<u64> = m.take_traced().iter().map(|(e, _)| e.ssv2_frame).collect();
        assert_eq!(traced, vec![2, 2]);
        // A new episode forgets the old map.
        m.begin_episode();
        m.queue.lock().unwrap().push(RawCollision {
            carla_frame: 500,
            ..hit(0, 0.1, Some(7), 1.0)
        });
        m.drain(|_| None);
        assert_eq!(m.take_traced()[0].0.ssv2_frame, 0);
    }
}
