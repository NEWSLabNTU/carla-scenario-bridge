//! The gap trace: SSv2's world and CARLA's, frame by frame, for the offline oracle
//! (roadmap 024).
//!
//! One JSON Lines file per `Initialize`, `<gap_trace_dir>/<UTC time>_<episode>.jsonl`, in
//! the format of `docs/design/gap-campaign.md`, appendix "Gap trace format" (schema 1). The
//! reader is `swb_launch/scripts/gap_trace.py`; a test here writes a trace and parses it
//! with that reader.
//!
//! # What a frame line pairs
//!
//! SSv2 frame `k` is the `UpdateEntityStatus` that precedes `UpdateFrame` `k` and the tick
//! that `UpdateFrame` runs. SSv2's world for it is the poses that `UpdateEntityStatus`
//! carried (an NPC's intended pose) and returned (the ego's, read back from CARLA); CARLA's
//! is the world after the tick, which is the snapshot the *next* `UpdateEntityStatus`
//! reads its tracked NPCs from. So the coordinator stages frame `k`'s SSv2 poses
//! ([`GapTrace::stage_frame`]) and writes the line one request later, from that next
//! snapshot ([`GapTrace::emit_staged`]): one snapshot per frame, shared with the tracked
//! path. A staged frame still open when an entity despawns or the episode ends is written
//! from a snapshot taken there.
//!
//! # Cost
//!
//! Nothing here touches the disk. Lines go to one writer thread over a bounded channel; a
//! frame line that does not fit is dropped and counted (the `end` line carries the count),
//! and the rarer lines a reader cannot do without (`episode`, `entity`, `contact`, `end`)
//! wait in a local backlog, in order, until the channel has room. The ZMQ thread never
//! blocks on the writer. With `gap_trace_dir` unset nothing is built at all.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError};

use serde::Serialize;

use crate::contact_report::{self, RawCollision};

/// The trace format version (`docs/design/gap-campaign.md`, appendix).
pub const SCHEMA: u32 = 1;

/// Lines the writer channel holds: about 25 s of frames at 20 Hz with a few entities, so
/// only a disk stalled for that long drops a frame.
pub const CHANNEL_LINES: usize = 512;

/// A pose in the ROS map frame: `[x, y, z, roll, pitch, yaw]`, m and rad.
pub type Pose6 = [f64; 6];

/// One record of the trace, serialized exactly as the schema names it.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Record {
    Episode {
        schema: u32,
        episode: u64,
        start_unix: f64,
        npc_motion: &'static str,
    },
    Entity(EntityHeader),
    Frame {
        frame: u64,
        sim_time: f64,
        /// The CARLA frame of the snapshot CARLA's half was read from: the frame's last tick
        /// (roadmap 032 N2), so a contact's `carla_frame` can be checked against the SSv2
        /// frame it is labelled with. Absent when unknown; older traces have none.
        #[serde(skip_serializing_if = "Option::is_none")]
        carla_frame: Option<u64>,
        entities: BTreeMap<String, EntityState>,
    },
    Contact {
        frame: u64,
        carla_frame: u64,
        sim_time: f64,
        ego_id: u32,
        /// The tracked NPC whose own sensor saw the contact (roadmap 032 step 5); absent for
        /// the ego's sensor, so an ego line is written exactly as before.
        #[serde(skip_serializing_if = "Option::is_none")]
        subject: Option<String>,
        other_id: Option<u32>,
        name: Option<String>,
        other_type: String,
        other_kind: &'static str,
        impulse: f64,
    },
    End {
        frame: u64,
        sim_time: f64,
        frames: u64,
        dropped_frames: u64,
        complete: bool,
    },
}

impl Record {
    pub fn to_line(&self) -> String {
        // Every field is a plain number, string or list: serialization cannot fail.
        serde_json::to_string(self).expect("trace records serialize")
    }
}

/// An entity's kind as the trace names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Ego,
    Vehicle,
    Pedestrian,
    MiscObject,
}

impl Kind {
    pub fn as_str(self) -> &'static str {
        match self {
            Kind::Ego => "ego",
            Kind::Vehicle => "vehicle",
            Kind::Pedestrian => "pedestrian",
            Kind::MiscObject => "misc_object",
        }
    }
}

impl Serialize for Kind {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(self.as_str())
    }
}

/// The declared box: `bounding_box.center` and `dimensions`, entity frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct DeclaredBox {
    pub center: [f64; 3],
    pub dimensions: [f64; 3],
}

/// CARLA's box in the actor's own frame, ROS-handed (y left).
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct BlueprintBox {
    pub location: [f64; 3],
    pub extent: [f64; 3],
}

/// The `entity` record, written once per spawn.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct EntityHeader {
    pub frame: u64,
    pub name: String,
    pub kind: Kind,
    pub declared: DeclaredBox,
    /// `[x, y, yaw]`, the bridge's `OriginOffset`.
    pub origin_offset: [f64; 3],
    pub actor_id: u32,
    pub blueprint: String,
    pub blueprint_box: Option<BlueprintBox>,
}

/// One entity in one frame line.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct EntityState {
    pub ssv2: Pose6,
    pub carla: Option<Pose6>,
    pub velocity: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub flags: Vec<&'static str>,
}

/// What CARLA's snapshot says about one actor: transform and velocity, map frame.
pub type CarlaState = (Pose6, [f64; 3]);

/// One entity of a staged frame: what SSv2 had, before CARLA's half is known.
#[derive(Debug, Clone, PartialEq)]
pub struct StagedEntity {
    pub name: String,
    pub actor_id: u32,
    pub ssv2: Pose6,
    /// A tracked NPC within `CONTACT_HOLD_FRAMES` of an ego contact (roadmap 023).
    pub contact_hold: bool,
    /// A tracked NPC handed to kinematic after a blocked snap (roadmap 032): physics off,
    /// at SSv2's pose.
    pub kinematic: bool,
}

#[derive(Debug, Clone)]
struct Staged {
    frame: u64,
    sim_time: f64,
    entities: Vec<StagedEntity>,
}

/// What the writer thread is told.
#[derive(Debug)]
enum Msg {
    Open(PathBuf),
    Line(String),
    Close,
}

/// The episode being recorded.
#[derive(Debug)]
struct Episode {
    episode: u64,
    frames: u64,
    dropped: u64,
    last_frame: u64,
    last_sim_time: f64,
    complete: bool,
    ego_id: Option<u32>,
    /// Entity name to kind, from the headers, for contact lines.
    kinds: HashMap<String, Kind>,
    /// Entities that have been in a frame line since their header: the rest get `spawn`.
    seen: HashSet<String>,
    staged: Option<Staged>,
}

/// The recorder as the coordinator holds it.
pub struct GapTrace {
    dir: Option<PathBuf>,
    tx: Option<SyncSender<Msg>>,
    writer: Option<std::thread::JoinHandle<()>>,
    /// Lines that must not be dropped and did not fit, oldest first.
    backlog: VecDeque<Msg>,
    episode: Option<Episode>,
    /// Set after a warning about records arriving with no episode open.
    warned_closed: bool,
}

impl GapTrace {
    /// `dir`: where traces go; `None` records nothing.
    pub fn new(dir: Option<PathBuf>) -> Self {
        match &dir {
            Some(d) => tracing::info!("Gap trace on: {}", d.display()),
            None => tracing::debug!("Gap trace off (gap_trace_dir unset)"),
        }
        Self {
            dir,
            tx: None,
            writer: None,
            backlog: VecDeque::new(),
            episode: None,
            warned_closed: false,
        }
    }

    /// A recorder over a channel the caller holds the other end of (tests).
    #[cfg(test)]
    fn with_channel(dir: PathBuf, tx: SyncSender<Msg>) -> Self {
        Self {
            dir: Some(dir),
            tx: Some(tx),
            writer: None,
            backlog: VecDeque::new(),
            episode: None,
            warned_closed: false,
        }
    }

    pub fn is_on(&self) -> bool {
        self.dir.is_some()
    }

    /// Whether frames are being recorded now (on, and an episode open).
    pub fn recording(&self) -> bool {
        self.episode.is_some()
    }

    /// A new `Initialize`: end the previous episode if still open, open the next file.
    pub fn begin_episode(&mut self, episode: u64, npc_motion: &'static str, start_unix: f64) {
        let Some(dir) = self.dir.clone() else {
            return;
        };
        self.end_episode();
        if self.tx.is_none() {
            self.start_writer();
        }
        let path = dir.join(format!(
            "{}_{episode}.jsonl",
            contact_report::utc_stamp(start_unix)
        ));
        self.push_control(Msg::Open(path));
        self.episode = Some(Episode {
            episode,
            frames: 0,
            dropped: 0,
            last_frame: 0,
            last_sim_time: 0.0,
            complete: true,
            ego_id: None,
            kinds: HashMap::new(),
            seen: HashSet::new(),
            staged: None,
        });
        self.warned_closed = false;
        self.push_line(Record::Episode {
            schema: SCHEMA,
            episode,
            start_unix,
            npc_motion,
        });
    }

    /// An entity spawned. A respawn under the same name replaces the earlier header.
    pub fn entity(&mut self, header: EntityHeader) {
        let Some(ep) = self.open_episode() else {
            return;
        };
        if header.kind == Kind::Ego {
            ep.ego_id = Some(header.actor_id);
        }
        ep.kinds.insert(header.name.clone(), header.kind);
        ep.seen.remove(&header.name);
        self.push_line(Record::Entity(header));
    }

    /// SSv2's half of frame `frame` (see the module docs). Replaces a staged frame that was
    /// never ticked.
    pub fn stage_frame(&mut self, frame: u64, entities: Vec<StagedEntity>) {
        let Some(ep) = self.episode.as_mut() else {
            return;
        };
        let sim_time = ep.last_sim_time;
        ep.staged = Some(Staged {
            frame,
            sim_time,
            entities,
        });
    }

    /// SSv2's simulation time for frame `frame`, from its `UpdateFrame`.
    pub fn frame_time(&mut self, frame: u64, sim_time: f64) {
        if let Some(staged) = self
            .episode
            .as_mut()
            .and_then(|ep| ep.staged.as_mut())
            .filter(|s| s.frame == frame)
        {
            staged.sim_time = if sim_time.is_finite() {
                sim_time.max(0.0)
            } else {
                0.0
            };
        }
    }

    /// Whether a staged frame has been ticked by `now` (the current SSv2 frame) and waits
    /// for CARLA's half.
    pub fn staged_due(&self, now: u64) -> bool {
        self.episode
            .as_ref()
            .and_then(|ep| ep.staged.as_ref())
            .is_some_and(|s| s.frame <= now)
    }

    /// Write the staged frame if it is due by `now`, with CARLA's state from `carla_of`
    /// (`None`: the actor is not in the snapshot).
    pub fn emit_staged<F>(&mut self, now: u64, carla_of: F)
    where
        F: FnMut(u32) -> Option<CarlaState>,
    {
        self.emit_staged_at(now, None, carla_of);
    }

    /// [`GapTrace::emit_staged`] from the snapshot of CARLA frame `carla_frame`.
    pub fn emit_staged_at<F>(&mut self, now: u64, carla_frame: Option<u64>, mut carla_of: F)
    where
        F: FnMut(u32) -> Option<CarlaState>,
    {
        if !self.staged_due(now) {
            return;
        }
        let ep = self.episode.as_mut().expect("staged_due checked it");
        let staged = ep.staged.take().expect("staged_due checked it");
        let mut entities = BTreeMap::new();
        for e in staged.entities {
            let carla = carla_of(e.actor_id);
            let mut flags = Vec::new();
            if ep.seen.insert(e.name.clone()) {
                flags.push("spawn");
            }
            if e.contact_hold {
                flags.push("contact_hold");
            }
            if e.kinematic {
                flags.push("kinematic");
            }
            entities.insert(
                e.name,
                EntityState {
                    ssv2: e.ssv2,
                    carla: carla.map(|c| c.0),
                    velocity: carla.map(|c| c.1),
                    flags,
                },
            );
        }
        let record = Record::Frame {
            frame: staged.frame,
            sim_time: staged.sim_time,
            carla_frame,
            entities,
        };
        if self.try_frame(record.to_line()) {
            let ep = self.episode.as_mut().expect("open");
            ep.frames += 1;
            ep.last_frame = staged.frame;
            ep.last_sim_time = staged.sim_time;
        } else {
            let ep = self.episode.as_mut().expect("open");
            ep.dropped += 1;
            ep.last_frame = staged.frame;
            ep.last_sim_time = staged.sim_time;
            if ep.dropped == 1 {
                tracing::warn!(
                    "Gap trace: writer behind, frame {} dropped (the disk is slow; the count \
                     goes in the trace's end line)",
                    staged.frame
                );
            }
        }
    }

    /// Contacts `collision_monitor` drained, with the other actor's SSv2 name when known:
    /// the ego's sensor's, and tracked NPCs' own (`subject`, roadmap 032 step 5).
    pub fn contacts(&mut self, events: Vec<(RawCollision, Option<String>)>) {
        if events.is_empty() {
            return;
        }
        let Some(ep) = self.open_episode() else {
            return;
        };
        let ego_id = ep.ego_id.unwrap_or(0);
        let records: Vec<Record> = events
            .into_iter()
            .map(|(event, name)| {
                let other_kind = match (event.other_id, &name) {
                    (None, _) => "static",
                    (Some(_), Some(n)) => ep.kinds.get(n).map_or("unknown", |k| k.as_str()),
                    (Some(_), None) => "unknown",
                };
                Record::Contact {
                    frame: event.ssv2_frame,
                    carla_frame: event.carla_frame,
                    sim_time: event.sim_time.max(0.0),
                    ego_id,
                    subject: event.subject,
                    other_id: event.other_id,
                    name,
                    other_type: event.other_type,
                    other_kind,
                    impulse: event.impulse.max(0.0),
                }
            })
            .collect();
        for r in records {
            self.push_line(r);
        }
    }

    /// CARLA restarted: contacts after this point were not seen.
    pub fn mark_incomplete(&mut self) {
        if let Some(ep) = self.episode.as_mut() {
            ep.complete = false;
        }
    }

    /// Close the episode: the `end` line, then the file. A staged frame still unwritten is
    /// dropped silently here -- the caller writes it first when it can.
    pub fn end_episode(&mut self) {
        let Some(ep) = self.episode.take() else {
            return;
        };
        self.push_line(Record::End {
            frame: ep.last_frame,
            sim_time: ep.last_sim_time,
            frames: ep.frames,
            dropped_frames: ep.dropped,
            complete: ep.complete,
        });
        self.push_control(Msg::Close);
        tracing::info!(
            "Gap trace: episode {} ended, {} frame(s), {} dropped",
            ep.episode,
            ep.frames,
            ep.dropped
        );
    }

    /// End the open episode and wait (up to `wait`) for the writer to finish. For process
    /// shutdown, where blocking is allowed.
    pub fn shutdown(&mut self, wait: std::time::Duration) {
        self.end_episode();
        if let Some(tx) = self.tx.take() {
            for msg in self.backlog.drain(..) {
                if tx.send(msg).is_err() {
                    break;
                }
            }
            drop(tx);
        }
        if let Some(handle) = self.writer.take() {
            let deadline = std::time::Instant::now() + wait;
            while !handle.is_finished() && std::time::Instant::now() < deadline {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            if handle.is_finished() {
                let _ = handle.join();
            } else {
                tracing::warn!(
                    "Gap trace: writer still busy at shutdown; the last lines may be lost"
                );
            }
        }
    }

    // --- channel ------------------------------------------------------------------------

    fn open_episode(&mut self) -> Option<&mut Episode> {
        if self.episode.is_none() {
            if self.dir.is_some() && !self.warned_closed {
                self.warned_closed = true;
                tracing::warn!(
                    "Gap trace: a record arrived after the episode's end line (an entity \
                     spawned after every entity was despawned?); not recorded"
                );
            }
            return None;
        }
        self.episode.as_mut()
    }

    fn start_writer(&mut self) {
        let (tx, rx) = std::sync::mpsc::sync_channel(CHANNEL_LINES);
        match std::thread::Builder::new()
            .name("gap-trace".into())
            .spawn(move || writer_loop(rx))
        {
            Ok(handle) => {
                self.tx = Some(tx);
                self.writer = Some(handle);
            }
            Err(e) => {
                tracing::warn!("Gap trace: could not start the writer thread ({e}); off");
                self.dir = None;
            }
        }
    }

    /// Send what waits in the backlog; true when it is empty.
    fn flush_backlog(&mut self) -> bool {
        let Some(tx) = self.tx.as_ref() else {
            self.backlog.clear();
            return true;
        };
        while let Some(msg) = self.backlog.pop_front() {
            match tx.try_send(msg) {
                Ok(()) => {}
                Err(TrySendError::Full(msg)) => {
                    self.backlog.push_front(msg);
                    return false;
                }
                Err(TrySendError::Disconnected(_)) => {
                    self.backlog.clear();
                    return true;
                }
            }
        }
        true
    }

    /// A line that must arrive: sent now, or kept in order for later.
    fn push_control(&mut self, msg: Msg) {
        self.backlog.push_back(msg);
        self.flush_backlog();
    }

    fn push_line(&mut self, record: Record) {
        self.push_control(Msg::Line(record.to_line()));
    }

    /// A frame line: sent now, behind the backlog, or dropped. True when sent.
    fn try_frame(&mut self, line: String) -> bool {
        if !self.flush_backlog() {
            return false;
        }
        match self.tx.as_ref().map(|tx| tx.try_send(Msg::Line(line))) {
            Some(Ok(())) => true,
            Some(Err(_)) | None => false,
        }
    }
}

impl Drop for GapTrace {
    fn drop(&mut self) {
        if self.is_on() {
            self.shutdown(std::time::Duration::from_secs(2));
        }
    }
}

/// The writer thread: one file open at a time, flushed whenever the channel runs dry.
fn writer_loop(rx: Receiver<Msg>) {
    let mut file: Option<(PathBuf, std::io::BufWriter<std::fs::File>)> = None;
    let mut failed: Option<PathBuf> = None;
    let close = |file: &mut Option<(PathBuf, std::io::BufWriter<std::fs::File>)>| {
        if let Some((path, mut w)) = file.take() {
            match w.flush() {
                Ok(()) => tracing::info!("Gap trace written: {}", path.display()),
                Err(e) => tracing::warn!("Gap trace {}: {e}", path.display()),
            }
        }
    };
    while let Ok(mut msg) = rx.recv() {
        loop {
            match msg {
                Msg::Open(path) => {
                    close(&mut file);
                    failed = None;
                    match open(&path) {
                        Ok(w) => file = Some((path, w)),
                        Err(e) => {
                            tracing::warn!("Gap trace: cannot create {} ({e})", path.display());
                            failed = Some(path);
                        }
                    }
                }
                Msg::Line(line) => {
                    if let Some((path, w)) = file.as_mut() {
                        if let Err(e) = writeln!(w, "{line}") {
                            tracing::warn!(
                                "Gap trace {}: {e}; rest of episode lost",
                                path.display()
                            );
                            failed = file.take().map(|(p, _)| p);
                        }
                    } else if failed.is_none() {
                        tracing::debug!("Gap trace: line with no file open");
                    }
                }
                Msg::Close => close(&mut file),
            }
            match rx.try_recv() {
                Ok(next) => msg = next,
                Err(_) => break,
            }
        }
        if let Some((_, w)) = file.as_mut() {
            let _ = w.flush();
        }
    }
    close(&mut file);
}

fn open(path: &Path) -> std::io::Result<std::io::BufWriter<std::fs::File>> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    Ok(std::io::BufWriter::new(std::fs::File::create(path)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    fn parse(r: &Record) -> Value {
        serde_json::from_str(&r.to_line()).expect("valid JSON")
    }

    fn header(name: &str, kind: Kind, actor_id: u32) -> EntityHeader {
        EntityHeader {
            frame: 0,
            name: name.into(),
            kind,
            declared: DeclaredBox {
                center: [1.5, 0.0, 0.75],
                dimensions: [4.5, 2.0, 1.5],
            },
            origin_offset: [1.5, 0.0, 0.0],
            actor_id,
            blueprint: "vehicle.tesla.model3".into(),
            blueprint_box: Some(BlueprintBox {
                location: [0.0, 0.0, 0.72],
                extent: [2.35, 1.05, 0.72],
            }),
        }
    }

    fn hit(frame: u64, other: Option<u32>) -> RawCollision {
        RawCollision {
            ssv2_frame: frame,
            carla_frame: 100 + frame,
            sim_time: 0.05 * frame as f64,
            other_id: other,
            other_type: if other.is_some() {
                "vehicle.tesla.model3".into()
            } else {
                "static.pole".into()
            },
            impulse: 30.0,
            subject: None,
        }
    }

    /// Roadmap 032 step 5: a tracked NPC's own sensor. `subject` names it; a line without
    /// one is the ego's, written exactly as before.
    #[test]
    fn an_npc_sensor_contact_names_its_subject() {
        let (tx, rx) = std::sync::mpsc::sync_channel(100);
        let mut t = GapTrace::with_channel(PathBuf::from("/nonexistent"), tx);
        t.begin_episode(1, "tracked", 0.0);
        t.entity(header("ego", Kind::Ego, 10));
        t.entity(header("npc_05", Kind::Vehicle, 11));
        t.entity(header("npc_09", Kind::Vehicle, 12));
        let npc = |other: Option<u32>| RawCollision {
            subject: Some("npc_05".into()),
            ..hit(25, other)
        };
        t.contacts(vec![
            (npc(None), None),
            (npc(Some(12)), Some("npc_09".into())),
            (npc(Some(10)), Some("ego".into())),
            (hit(25, None), None),
        ]);
        let v = lines(&rx);
        let contacts: Vec<&Value> = v.iter().filter(|l| l["type"] == "contact").collect();
        assert_eq!(contacts.len(), 4);
        assert_eq!(contacts[0]["subject"], "npc_05");
        assert_eq!(contacts[0]["other_kind"], "static");
        assert_eq!(contacts[0]["ego_id"], 10);
        assert_eq!(contacts[1]["other_kind"], "vehicle");
        assert_eq!(contacts[2]["other_kind"], "ego");
        assert!(contacts[3].get("subject").is_none());
    }

    #[test]
    fn episode_record() {
        let r = Record::Episode {
            schema: SCHEMA,
            episode: 3,
            start_unix: 1_760_000_000.25,
            npc_motion: "tracked",
        };
        assert_eq!(
            parse(&r),
            json!({"type": "episode", "schema": 1, "episode": 3,
                   "start_unix": 1_760_000_000.25, "npc_motion": "tracked"})
        );
    }

    #[test]
    fn entity_record() {
        let mut h = header("truck", Kind::Vehicle, 212);
        assert_eq!(
            parse(&Record::Entity(h.clone())),
            json!({"type": "entity", "frame": 0, "name": "truck", "kind": "vehicle",
                   "declared": {"center": [1.5, 0.0, 0.75], "dimensions": [4.5, 2.0, 1.5]},
                   "origin_offset": [1.5, 0.0, 0.0], "actor_id": 212,
                   "blueprint": "vehicle.tesla.model3",
                   "blueprint_box": {"location": [0.0, 0.0, 0.72], "extent": [2.35, 1.05, 0.72]}})
        );
        h.kind = Kind::MiscObject;
        h.blueprint_box = None;
        let v = parse(&Record::Entity(h));
        assert_eq!(v["kind"], "misc_object");
        assert_eq!(v["blueprint_box"], Value::Null);
    }

    #[test]
    fn frame_record_omits_empty_flags_and_nulls_a_missing_actor() {
        let mut entities = BTreeMap::new();
        entities.insert(
            "ego".to_string(),
            EntityState {
                ssv2: [1.0, 2.0, 0.0, 0.0, 0.0, 0.5],
                carla: Some([1.1, 2.0, 0.03, 0.0, 0.0, 0.5]),
                velocity: Some([4.0, 0.0, 0.0]),
                flags: vec![],
            },
        );
        entities.insert(
            "npc".to_string(),
            EntityState {
                ssv2: [5.0, 2.0, 0.0, 0.0, 0.0, 0.0],
                carla: None,
                velocity: None,
                flags: vec!["spawn", "contact_hold"],
            },
        );
        let r = Record::Frame {
            frame: 7,
            sim_time: 0.35,
            carla_frame: None,
            entities,
        };
        assert_eq!(
            parse(&r),
            json!({"type": "frame", "frame": 7, "sim_time": 0.35, "entities": {
                "ego": {"ssv2": [1.0, 2.0, 0.0, 0.0, 0.0, 0.5],
                        "carla": [1.1, 2.0, 0.03, 0.0, 0.0, 0.5], "velocity": [4.0, 0.0, 0.0]},
                "npc": {"ssv2": [5.0, 2.0, 0.0, 0.0, 0.0, 0.0], "carla": null,
                        "velocity": null, "flags": ["spawn", "contact_hold"]}}})
        );
    }

    #[test]
    fn contact_and_end_records() {
        let r = Record::Contact {
            frame: 1,
            carla_frame: 5021,
            sim_time: 0.05,
            ego_id: 211,
            other_id: None,
            name: None,
            other_type: "static.pole".into(),
            other_kind: "static",
            impulse: 30.0,
            subject: None,
        };
        assert_eq!(
            parse(&r),
            json!({"type": "contact", "frame": 1, "carla_frame": 5021, "sim_time": 0.05,
                   "ego_id": 211, "other_id": null, "name": null, "other_type": "static.pole",
                   "other_kind": "static", "impulse": 30.0})
        );
        let r = Record::End {
            frame: 9,
            sim_time: 0.45,
            frames: 9,
            dropped_frames: 1,
            complete: false,
        };
        assert_eq!(
            parse(&r),
            json!({"type": "end", "frame": 9, "sim_time": 0.45, "frames": 9,
                   "dropped_frames": 1, "complete": false})
        );
    }

    fn lines(rx: &Receiver<Msg>) -> Vec<Value> {
        let mut out = Vec::new();
        while let Ok(msg) = rx.try_recv() {
            if let Msg::Line(l) = msg {
                out.push(serde_json::from_str(&l).unwrap());
            }
        }
        out
    }

    fn staged(name: &str, actor_id: u32, x: f64, hold: bool) -> StagedEntity {
        StagedEntity {
            name: name.into(),
            actor_id,
            ssv2: [x, 0.0, 0.0, 0.0, 0.0, 0.0],
            contact_hold: hold,
            kinematic: false,
        }
    }

    #[test]
    fn frames_are_paired_one_request_later_flag_spawn_once_and_name_contacts() {
        let (tx, rx) = std::sync::mpsc::sync_channel(100);
        let mut t = GapTrace::with_channel(PathBuf::from("/nonexistent"), tx);
        t.begin_episode(1, "tracked", 1_760_000_000.0);
        t.entity(header("ego", Kind::Ego, 10));
        t.entity(header("npc", Kind::Vehicle, 11));
        // UpdateEntityStatus 1 stages frame 1; UpdateFrame 1 stamps its time.
        t.stage_frame(
            1,
            vec![staged("ego", 10, 0.0, false), staged("npc", 11, 5.0, false)],
        );
        t.frame_time(1, 0.05);
        // Not due before its tick.
        t.emit_staged(0, |_| panic!("not due"));
        t.emit_staged(1, |id| {
            (id == 10).then_some(([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0]))
        });
        t.stage_frame(
            2,
            vec![
                staged("ego", 10, 0.1, false),
                StagedEntity {
                    kinematic: true,
                    ..staged("npc", 11, 5.2, true)
                },
            ],
        );
        t.frame_time(2, 0.10);
        t.contacts(vec![
            (hit(2, Some(11)), Some("npc".into())),
            (hit(2, None), None),
            (hit(2, Some(99)), None),
        ]);
        t.emit_staged_at(2, Some(5021), |_| Some(([0.0; 6], [0.0; 3])));
        // Staged but never ticked: replaced, then left at the end.
        t.stage_frame(3, vec![]);
        t.end_episode();

        let v = lines(&rx);
        let types: Vec<&str> = v.iter().map(|l| l["type"].as_str().unwrap()).collect();
        assert_eq!(
            types,
            [
                "episode", "entity", "entity", "frame", "contact", "contact", "contact", "frame",
                "end"
            ]
        );
        assert_eq!(v[3]["frame"], 1);
        assert_eq!(v[3]["sim_time"], 0.05);
        assert_eq!(v[3]["entities"]["ego"]["flags"], json!(["spawn"]));
        assert_eq!(v[3]["entities"]["npc"]["carla"], Value::Null);
        // The snapshot's CARLA frame when known (032 N2), and no key otherwise.
        assert!(v[3].get("carla_frame").is_none());
        assert_eq!(v[7]["carla_frame"], 5021);
        assert_eq!(v[4]["other_kind"], "vehicle");
        assert_eq!(v[4]["ego_id"], 10);
        assert_eq!(v[5]["other_kind"], "static");
        assert_eq!(v[6]["other_kind"], "unknown");
        assert!(v[7]["entities"]["ego"].get("flags").is_none());
        assert_eq!(
            v[7]["entities"]["npc"]["flags"],
            json!(["contact_hold", "kinematic"])
        );
        assert_eq!(
            v[8],
            json!({"type": "end", "frame": 2, "sim_time": 0.1, "frames": 2,
                                "dropped_frames": 0, "complete": true})
        );
    }

    #[test]
    fn a_full_channel_drops_frames_but_keeps_the_other_lines_in_order() {
        // Room for two messages and nobody reading: the ZMQ thread must never wait.
        let (tx, rx) = std::sync::mpsc::sync_channel(2);
        let mut t = GapTrace::with_channel(PathBuf::from("/nonexistent"), tx);
        t.begin_episode(4, "kinematic", 0.0); // Open + episode: full
        t.entity(header("ego", Kind::Ego, 1)); // backlog
        for f in 1..=3 {
            t.stage_frame(f, vec![staged("ego", 1, 0.0, false)]);
            t.emit_staged(f, |_| None); // dropped: the backlog goes first
        }
        assert!(matches!(rx.try_recv(), Ok(Msg::Open(_))));
        assert!(matches!(rx.try_recv(), Ok(Msg::Line(_))));
        // Room now: the header goes, then frame 4 behind it.
        t.stage_frame(4, vec![staged("ego", 1, 0.0, false)]);
        t.emit_staged(4, |_| None);
        let v = lines(&rx);
        assert_eq!(v[0]["type"], "entity");
        assert_eq!(v[1]["frame"], 4);
        // `spawn` marks the spawn frame, which was dropped: no later line inherits it.
        assert!(v[1]["entities"]["ego"].get("flags").is_none());
        t.end_episode();
        let v = lines(&rx);
        assert_eq!(v[0]["type"], "end");
        assert_eq!(v[0]["frames"], 1);
        assert_eq!(v[0]["dropped_frames"], 3);
    }

    #[test]
    fn off_records_nothing() {
        let mut t = GapTrace::new(None);
        t.begin_episode(1, "tracked", 0.0);
        t.entity(header("ego", Kind::Ego, 1));
        t.stage_frame(1, vec![staged("ego", 1, 0.0, false)]);
        assert!(!t.recording());
        assert!(!t.staged_due(5));
        assert!(t.tx.is_none() && t.writer.is_none());
    }

    /// The format is the Python reader's: write a trace through the real writer thread and
    /// parse it with `swb_launch/scripts/gap_trace.py`. Cross-language in one test, so a
    /// field renamed on either side fails here without a golden file to keep in step.
    #[test]
    fn a_written_trace_parses_with_the_python_reader() {
        let dir = std::env::temp_dir().join(format!("swb-gap-trace-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut t = GapTrace::new(Some(dir.clone()));
        t.begin_episode(2, "tracked", 1_791_683_599.572);
        t.entity(header("ego", Kind::Ego, 10));
        let mut prop = header("barrier", Kind::MiscObject, 12);
        prop.blueprint_box = None;
        t.entity(prop);
        for f in 1..=5u64 {
            t.stage_frame(
                f,
                vec![
                    staged("ego", 10, f as f64, false),
                    StagedEntity {
                        kinematic: f == 4,
                        ..staged("barrier", 12, 20.0, f == 3)
                    },
                ],
            );
            t.frame_time(f, 0.05 * f as f64);
            // A CARLA frame on every frame but the first (032 N2: optional).
            t.emit_staged_at(f, (f > 1).then_some(5000 + f), |id| {
                (id == 10).then_some(([f as f64 + 0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0]))
            });
        }
        t.contacts(vec![
            (hit(4, None), None),
            (
                RawCollision {
                    subject: Some("barrier".into()),
                    ..hit(4, None)
                },
                None,
            ),
        ]);
        t.shutdown(std::time::Duration::from_secs(5));
        let path = dir.join("20261011T015319.572Z_2.jsonl");
        assert!(path.exists(), "{} not written", path.display());

        let python = match std::process::Command::new("python3")
            .arg("--version")
            .output()
        {
            Ok(o) if o.status.success() => "python3",
            _ => {
                eprintln!(
                    "SKIPPED: python3 not found; the trace was not checked against gap_trace.py"
                );
                return;
            }
        };
        let scripts = Path::new(env!("CARGO_MANIFEST_DIR")).join("../swb_launch/scripts");
        let script = r#"
import sys
sys.path.insert(0, sys.argv[1])
import gap_trace
t = gap_trace.load_trace(sys.argv[2])
assert t.episode.episode == 2 and t.episode.npc_motion == "tracked", t.episode
assert sorted(t.entities) == ["barrier", "ego"], t.entities
assert t.entities["barrier"].blueprint_box is None
assert [f.frame for f in t.frames] == [1, 2, 3, 4, 5]
assert [f.carla_frame for f in t.frames] == [None, 5002, 5003, 5004, 5005]
assert t.frames[0].entities["ego"].spawn and not t.frames[1].entities["ego"].spawn
assert t.frames[2].entities["barrier"].contact_hold
assert t.frames[3].entities["barrier"].kinematic and not t.frames[2].entities["barrier"].kinematic
assert t.frames[0].entities["barrier"].carla is None
assert abs(t.frames[4].entities["ego"].carla.x - 5.1) < 1e-9
assert len(t.contacts) == 2 and t.contacts[0].other_kind == "static" and t.contacts[0].ego_id == 10
assert t.contacts[0].subject is None and t.contacts[1].subject == "barrier"
assert t.end.frames == 5 and t.end.dropped_frames == 0 and t.end.complete
print("ok")
"#;
        let out = std::process::Command::new(python)
            .arg("-c")
            .arg(script)
            .arg(&scripts)
            .arg(&path)
            .output()
            .expect("run python3");
        let _ = std::fs::remove_dir_all(&dir);
        assert!(
            out.status.success(),
            "gap_trace.py rejected the trace:\n{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
    }
}
