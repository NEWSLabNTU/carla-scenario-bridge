//! Entities whose OpenSCENARIO controller is `agent`: csb as a *commander* of the agent
//! relay (docs/design/user-workflow.md, "The agent protocol (I3)", "Commanders").
//!
//! Such an entity is a CARLA vehicle spawned with `role_name` = the entity name, so the
//! vehicle side started with `vehicle_name:=<entity>` attaches to it, and driven by the
//! agent that registered with the relay under the entity name -- typically a full Autoware
//! behind `acb_pilot agent`. SSv2 does not move it (the fork's SimulatorDrivenVehicleEntity,
//! as for `simulator_autopilot`); csb reads its pose back from CARLA, and forwards what the
//! scenario asks of it to its agent:
//!
//! | scenario / SSv2                         | agent command      |
//! |-----------------------------------------|--------------------|
//! | spawn (its pose)                        | `teleported`       |
//! | `AcquirePositionAction` / `AssignRoute` | `set_goal`         |
//! | `cancelRequest` (route cleared)         | `clear_goal`       |
//! | absolute `SpeedAction`                  | `set_speed_limit`  |
//! | despawn, next `Initialize`, shutdown    | `stop`             |
//!
//! None of this blocks a frame. Commands go out from a worker thread in order per entity:
//! an agent re-localizing after `teleported` refuses a goal (INITIALIZING), so a goal is
//! held until SSv2's NPC logic has started (like SSv2's own NPCs and `simulator_autopilot`)
//! and the agent reports a phase that takes goals. A refusal or a missing agent becomes the
//! entity's error, which the next `UpdateEntityStatus` reports as a failed frame, so SSv2
//! ends the scenario naming the entity. The one synchronous call is the registration check
//! in `UpdateEntityGoal` (`check_agent`), a relay lookup that answers from memory.
//!
//! Simulator-neutral apart from where it is called from: any simulator adapter could use
//! the same commander protocol.

use std::collections::{HashMap, VecDeque};
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use color_eyre::eyre::{self, bail, Result, WrapErr};
use serde_json::{json, Value};

/// The OpenSCENARIO controller name (SpawnVehicleEntityRequest.behavior) of these entities.
pub const BEHAVIOR: &str = "agent";

/// Agent protocol version (`"v"` on every message).
const VERSION: u64 = 1;
/// How long the relay waits for the agent's reply to a forwarded command. `set_goal` on
/// acb's agent can take a clear (2 s), a stop (2 s) and the route's settle (2 s).
const COMMAND_TIMEOUT_S: f64 = 10.0;
/// Our socket read timeout: the relay's command timeout plus margin.
const READ_TIMEOUT: Duration = Duration::from_secs(15);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
/// How often a held goal asks the relay for the agent's phase.
const POLL: Duration = Duration::from_millis(500);
/// How soon a `teleported` that found no agent (or no relay) is tried again.
const RETRY: Duration = Duration::from_secs(1);

/// Phases in which an agent takes a goal (`set_goal` fails while INITIALIZING/UNAVAILABLE).
pub fn takes_goals(phase: &str) -> bool {
    matches!(
        phase,
        "IDLE" | "PLANNING" | "READY" | "DRIVING" | "ARRIVED" | "STOPPED"
    )
}

/// A pose in the agent protocol: map frame (ROS), quaternion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AgentPose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
}

impl AgentPose {
    fn to_json(self) -> Value {
        json!({"x": self.x, "y": self.y, "z": self.z,
               "qx": self.qx, "qy": self.qy, "qz": self.qz, "qw": self.qw})
    }
}

/// One agent-protocol command for an entity's agent.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Teleported(AgentPose),
    SetGoal {
        goal: AgentPose,
        waypoints: Vec<AgentPose>,
    },
    ClearGoal,
    /// `None`: no limit.
    SetSpeedLimit(Option<f64>),
    Stop,
}

impl Command {
    pub fn name(&self) -> &'static str {
        match self {
            Command::Teleported(_) => "teleported",
            Command::SetGoal { .. } => "set_goal",
            Command::ClearGoal => "clear_goal",
            Command::SetSpeedLimit(_) => "set_speed_limit",
            Command::Stop => "stop",
        }
    }

    pub fn args(&self) -> Value {
        match self {
            Command::Teleported(pose) => json!({"pose": pose.to_json()}),
            Command::SetGoal { goal, waypoints } => json!({
                "goal": goal.to_json(),
                "waypoints": waypoints.iter().map(|w| w.to_json()).collect::<Vec<_>>(),
                "allow_goal_modification": false,
            }),
            Command::ClearGoal | Command::Stop => json!({}),
            Command::SetSpeedLimit(mps) => json!({"mps": mps}),
        }
    }

    /// A goal change: a newer one makes a queued, unsent one moot.
    fn is_goal(&self) -> bool {
        matches!(self, Command::SetGoal { .. } | Command::ClearGoal)
    }

    /// The commands SSv2's request maps to, in the order they go out (speed first, so the
    /// agent drives off under the limit). Waypoints in order, the last is the goal.
    pub fn from_goal_request(
        waypoints: &[AgentPose],
        clear: bool,
        target_speed: Option<f64>,
    ) -> Vec<Command> {
        let mut commands = Vec::new();
        if let Some(mps) = target_speed {
            commands.push(Command::SetSpeedLimit(Some(mps)));
        }
        if let Some((goal, via)) = waypoints.split_last() {
            commands.push(Command::SetGoal {
                goal: *goal,
                waypoints: via.to_vec(),
            });
        } else if clear {
            commands.push(Command::ClearGoal);
        }
        commands
    }
}

/// The relay's answer to a commander's `command_for` or `query`.
#[derive(Debug, Clone, PartialEq)]
pub struct Reply {
    pub status: String,
    pub message: String,
    /// An agent is registered under the entity's name.
    pub registered: bool,
    /// The agent's latest phase, when it has reported one.
    pub phase: Option<String>,
}

impl Reply {
    fn ok(&self) -> bool {
        self.status == "OK"
    }

    fn from_json(v: &Value) -> Result<Self> {
        let status = v
            .get("status")
            .and_then(Value::as_str)
            .ok_or_else(|| eyre::eyre!("reply without a status"))?
            .to_string();
        let phase = v
            .get("state")
            .and_then(|s| s.get("phase"))
            .and_then(Value::as_str)
            .map(str::to_string);
        Ok(Self {
            registered: v
                .get("registered")
                .and_then(Value::as_bool)
                .unwrap_or(phase.is_some()),
            message: v
                .get("message")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_string(),
            status,
            phase,
        })
    }
}

/// What the worker does next for one entity.
#[derive(Debug, Clone, PartialEq)]
pub enum Next {
    Send(Command),
    /// Ask the relay for the agent's phase (a goal is waiting for it).
    Query,
    Idle,
}

/// One entity's outgoing commands and what is known of its agent. Pure: the worker thread
/// drives it, the tests drive it by hand.
#[derive(Debug, Default)]
pub struct EntityQueue {
    queue: VecDeque<Command>,
    /// The agent's latest phase, from any reply.
    phase: Option<String>,
    /// Why this entity can no longer be driven; reported by `UpdateEntityStatus`.
    error: Option<String>,
    /// Despawned: drop it once its `stop` is out.
    retired: bool,
    not_before: Option<Instant>,
}

impl EntityQueue {
    pub fn push(&mut self, command: Command) {
        if command.is_goal() {
            self.queue.retain(|c| !c.is_goal());
        }
        if matches!(command, Command::SetSpeedLimit(_)) {
            self.queue
                .retain(|c| !matches!(c, Command::SetSpeedLimit(_)));
        }
        self.queue.push_back(command);
    }

    /// Despawned: whatever was queued is moot; tell the agent to stop.
    pub fn retire(&mut self) {
        self.queue.clear();
        self.queue.push_back(Command::Stop);
        self.retired = true;
        self.not_before = None;
    }

    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    pub fn phase(&self) -> Option<&str> {
        self.phase.as_deref()
    }

    pub fn done(&self) -> bool {
        self.retired && self.queue.is_empty()
    }

    pub fn next(&self, now: Instant, npc_logic_started: bool) -> Next {
        let Some(front) = self.queue.front() else {
            return Next::Idle;
        };
        if self.error.is_some() && *front != Command::Stop {
            return Next::Idle;
        }
        if self.not_before.is_some_and(|t| now < t) {
            return Next::Idle;
        }
        if let Command::SetGoal { .. } = front {
            if !npc_logic_started {
                return Next::Idle;
            }
            if !self.phase.as_deref().is_some_and(takes_goals) {
                return Next::Query;
            }
        }
        Next::Send(front.clone())
    }

    /// The relay answered `sent` (the front of the queue).
    pub fn on_reply(&mut self, entity: &str, sent: &Command, reply: &Reply, now: Instant) {
        if reply.phase.is_some() || !reply.registered {
            self.phase = reply.phase.clone();
        }
        if self.queue.front() != Some(sent) {
            return; // replaced while in flight (a despawn)
        }
        if reply.ok() {
            self.queue.pop_front();
            self.not_before = None;
            return;
        }
        match sent {
            // Fire and forget: a despawned entity's agent that is gone has nothing to stop.
            Command::Stop => {
                self.queue.pop_front();
            }
            // The vehicle side may come up after the scenario; it is told where it is
            // as soon as it registers.
            Command::Teleported(_) if !reply.registered => {
                self.not_before = Some(now + RETRY);
            }
            // Raced the agent back into INITIALIZING: wait for it again.
            Command::SetGoal { .. }
                if reply.registered && !reply.phase.as_deref().is_some_and(takes_goals) =>
            {
                self.not_before = Some(now + POLL);
            }
            _ if !reply.registered => {
                self.error = Some(format!(
                    "no agent is registered with the agent relay as '{entity}' (controller \
                     {BEHAVIOR}); start its vehicle side with entity:={entity}"
                ));
            }
            _ => {
                self.error = Some(format!(
                    "its agent answered {} with {}{}",
                    sent.name(),
                    reply.status,
                    if reply.message.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", reply.message)
                    }
                ));
            }
        }
    }

    /// The relay answered a `query`.
    pub fn on_query(&mut self, reply: &Reply, now: Instant) {
        self.phase = reply.phase.clone();
        self.not_before = Some(now + POLL);
    }

    /// The relay could not be reached (or the connection broke) while handling `next`.
    pub fn on_link_down(&mut self, next: &Next, why: &str, relay: &str, now: Instant) {
        match next {
            Next::Send(Command::Stop) => {
                self.queue.pop_front();
            }
            Next::Send(Command::Teleported(_)) | Next::Query => {
                self.not_before = Some(now + RETRY);
            }
            Next::Send(command) => {
                self.error = Some(format!(
                    "the agent relay at {relay} is unreachable for {}: {why}",
                    command.name()
                ));
            }
            Next::Idle => {}
        }
    }
}

/// `tcp://host:port` (or `host:port`; port 5560 when absent).
pub fn parse_relay_address(address: &str) -> Result<(String, u16)> {
    let rest = address.strip_prefix("tcp://").unwrap_or(address);
    if rest.is_empty() || rest.contains("://") {
        bail!("agent relay must be tcp://host:port, got '{address}'");
    }
    match rest.rsplit_once(':') {
        Some((host, port)) if !host.is_empty() => Ok((
            host.to_string(),
            port.parse()
                .wrap_err_with(|| format!("agent relay port in '{address}'"))?,
        )),
        Some(_) => bail!("agent relay must be tcp://host:port, got '{address}'"),
        None => Ok((rest.to_string(), 5560)),
    }
}

/// One blocking commander connection to the relay.
pub struct RelayConnection {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
    next_id: u64,
}

impl RelayConnection {
    pub fn connect(address: &str) -> Result<Self> {
        let (host, port) = parse_relay_address(address)?;
        let socket = (host.as_str(), port)
            .to_socket_addrs()
            .wrap_err_with(|| format!("resolve {host}"))?
            .next()
            .ok_or_else(|| eyre::eyre!("{host} resolves to nothing"))?;
        let stream = TcpStream::connect_timeout(&socket, CONNECT_TIMEOUT)
            .wrap_err_with(|| format!("connect to {address}"))?;
        stream.set_nodelay(true).ok();
        stream.set_read_timeout(Some(READ_TIMEOUT))?;
        let reader = BufReader::new(stream.try_clone()?);
        let mut connection = Self {
            stream,
            reader,
            next_id: 1,
        };
        connection.send(json!({"type": "register", "role": "commander",
                                "agent": "carla_scenario_bridge"}))?;
        let answer = loop {
            let message = connection.read_message()?;
            if message.get("type").and_then(Value::as_str) != Some("heartbeat") {
                break message;
            }
        };
        match answer.get("type").and_then(Value::as_str) {
            Some("registered") => Ok(connection),
            Some("error") => bail!(
                "relay refused the commander: {}",
                answer.get("reason").and_then(Value::as_str).unwrap_or("?")
            ),
            other => bail!("relay answered register with {other:?}"),
        }
    }

    fn send(&mut self, mut message: Value) -> Result<()> {
        message["v"] = json!(VERSION);
        let mut line = serde_json::to_vec(&message)?;
        line.push(b'\n');
        self.stream.write_all(&line).wrap_err("send to the relay")
    }

    fn read_message(&mut self) -> Result<Value> {
        let mut line = String::new();
        loop {
            line.clear();
            if self
                .reader
                .read_line(&mut line)
                .wrap_err("read from the relay")?
                == 0
            {
                bail!("the relay closed the connection");
            }
            if line.trim().is_empty() {
                continue;
            }
            let message: Value = serde_json::from_str(&line).wrap_err("relay sent bad JSON")?;
            if message.get("v").and_then(Value::as_u64) != Some(VERSION) {
                bail!("relay speaks protocol version {:?}", message.get("v"));
            }
            return Ok(message);
        }
    }

    fn request(&mut self, mut message: Value) -> Result<Reply> {
        let id = self.next_id;
        self.next_id += 1;
        message["id"] = json!(id);
        self.send(message)?;
        loop {
            let answer = self.read_message()?;
            match answer.get("type").and_then(Value::as_str) {
                Some("reply") if answer.get("id").and_then(Value::as_u64) == Some(id) => {
                    return Reply::from_json(&answer);
                }
                Some("error") => bail!(
                    "relay error: {}",
                    answer.get("reason").and_then(Value::as_str).unwrap_or("?")
                ),
                _ => {} // heartbeat, or a late reply to an abandoned request
            }
        }
    }

    pub fn command(&mut self, entity: &str, command: &Command) -> Result<Reply> {
        self.request(json!({"type": "command_for", "entity": entity,
                            "command": command.name(), "args": command.args(),
                            "timeout": COMMAND_TIMEOUT_S}))
    }

    pub fn query(&mut self, entity: &str) -> Result<Reply> {
        self.request(json!({"type": "query", "entity": entity}))
    }
}

#[derive(Default)]
struct Shared {
    entities: HashMap<String, EntityQueue>,
    npc_logic_started: bool,
    stopping: bool,
}

/// The commander side: one worker thread, one connection for it, one for the frame
/// thread's registration checks.
pub struct AgentLink {
    relay: String,
    shared: Arc<(Mutex<Shared>, Condvar)>,
    probe: Option<RelayConnection>,
    worker: Option<JoinHandle<()>>,
}

impl AgentLink {
    pub fn start(relay: &str) -> Self {
        let shared = Arc::new((Mutex::new(Shared::default()), Condvar::new()));
        let worker = {
            let shared = shared.clone();
            let relay = relay.to_string();
            std::thread::Builder::new()
                .name("agent-link".into())
                .spawn(move || run_worker(&relay, &shared))
                .ok()
        };
        tracing::info!("Agent-driven entities: commands go to the agent relay at {relay}");
        Self {
            relay: relay.to_string(),
            shared,
            probe: None,
            worker,
        }
    }

    fn with<R>(&self, f: impl FnOnce(&mut Shared) -> R) -> R {
        let (lock, wake) = &*self.shared;
        let mut shared = lock.lock().unwrap_or_else(|e| e.into_inner());
        let r = f(&mut shared);
        wake.notify_all();
        r
    }

    /// A fresh spawn: (re)start the entity's queue with `teleported` at its pose.
    pub fn spawned(&self, entity: &str, pose: AgentPose) {
        self.with(|s| {
            let mut queue = EntityQueue::default();
            queue.push(Command::Teleported(pose));
            s.entities.insert(entity.to_string(), queue);
        });
    }

    pub fn push(&self, entity: &str, commands: Vec<Command>) {
        self.with(|s| {
            let queue = s.entities.entry(entity.to_string()).or_default();
            for command in commands {
                queue.push(command);
            }
        });
    }

    pub fn despawned(&self, entity: &str) {
        self.with(|s| {
            if let Some(queue) = s.entities.get_mut(entity) {
                queue.retire();
            }
        });
    }

    /// Every entity is gone (Initialize, teardown): stop all agents.
    pub fn despawn_all(&self) {
        self.with(|s| {
            s.npc_logic_started = false;
            for queue in s.entities.values_mut() {
                queue.retire();
            }
        });
    }

    pub fn set_npc_logic_started(&self, started: bool) {
        let (lock, wake) = &*self.shared;
        let mut shared = lock.lock().unwrap_or_else(|e| e.into_inner());
        if shared.npc_logic_started != started {
            shared.npc_logic_started = started;
            wake.notify_all();
        }
    }

    /// Why `entity` can no longer be driven, if it cannot.
    pub fn error(&self, entity: &str) -> Option<String> {
        let (lock, _) = &*self.shared;
        let shared = lock.lock().unwrap_or_else(|e| e.into_inner());
        shared
            .entities
            .get(entity)
            .and_then(|q| q.error().map(str::to_string))
    }

    /// Is an agent registered for `entity`? Synchronous: the relay answers from memory.
    pub fn check_agent(&mut self, entity: &str) -> Result<()> {
        let reply = match self.probe_query(entity) {
            Ok(reply) => reply,
            Err(_) => {
                // One reconnect: the relay may have restarted since the last check.
                self.probe = None;
                self.probe_query(entity)
                    .wrap_err_with(|| format!("the agent relay at {} is unreachable", self.relay))?
            }
        };
        if !reply.registered {
            bail!(
                "no agent is registered with the agent relay ({}) as '{entity}' (controller \
                 {BEHAVIOR}); start its vehicle side with entity:={entity}",
                self.relay
            );
        }
        Ok(())
    }

    fn probe_query(&mut self, entity: &str) -> Result<Reply> {
        if self.probe.is_none() {
            self.probe = Some(RelayConnection::connect(&self.relay)?);
        }
        let result = self.probe.as_mut().expect("connected").query(entity);
        if result.is_err() {
            self.probe = None;
        }
        result
    }

    /// Stop every agent and wait (briefly) for the stops to go out.
    pub fn shutdown(&mut self) {
        self.despawn_all();
        self.with(|s| s.stopping = true);
        if let Some(worker) = self.worker.take() {
            let deadline = Instant::now() + Duration::from_secs(3);
            while !worker.is_finished() && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(20));
            }
            if worker.is_finished() {
                let _ = worker.join();
            }
        }
    }
}

impl Drop for AgentLink {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn run_worker(relay: &str, shared: &(Mutex<Shared>, Condvar)) {
    let (lock, wake) = shared;
    let mut connection: Option<RelayConnection> = None;
    let mut warned_down = false;
    loop {
        // Pick the next piece of work (round robin is unnecessary: an entity waiting on its
        // agent is Idle until its poll time, which lets the others through).
        let work = {
            let mut s = lock.lock().unwrap_or_else(|e| e.into_inner());
            s.entities.retain(|_, q| !q.done());
            if s.stopping && s.entities.is_empty() {
                return;
            }
            let now = Instant::now();
            let started = s.npc_logic_started;
            let mut names: Vec<&String> = s.entities.keys().collect();
            names.sort();
            let work =
                names
                    .into_iter()
                    .find_map(|name| match s.entities[name].next(now, started) {
                        Next::Idle => None,
                        next => Some((name.clone(), next)),
                    });
            if work.is_none() {
                let _ = wake
                    .wait_timeout(s, Duration::from_millis(100))
                    .map(|(guard, _)| drop(guard));
                continue;
            }
            work.expect("some")
        };
        let (entity, next) = work;

        if connection.is_none() {
            match RelayConnection::connect(relay) {
                Ok(c) => {
                    tracing::info!("Agent relay {relay}: connected as a commander");
                    warned_down = false;
                    connection = Some(c);
                }
                Err(e) => {
                    if !warned_down {
                        tracing::warn!("Agent relay {relay} unreachable: {e:#}");
                        warned_down = true;
                    }
                    let mut s = lock.lock().unwrap_or_else(|e| e.into_inner());
                    if let Some(q) = s.entities.get_mut(&entity) {
                        q.on_link_down(&next, &format!("{e:#}"), relay, Instant::now());
                    }
                    continue;
                }
            }
        }
        let c = connection.as_mut().expect("connected");
        let outcome = match &next {
            Next::Send(command) => c.command(&entity, command),
            Next::Query => c.query(&entity),
            Next::Idle => unreachable!(),
        };
        let now = Instant::now();
        let mut s = lock.lock().unwrap_or_else(|e| e.into_inner());
        let Some(queue) = s.entities.get_mut(&entity) else {
            continue;
        };
        match (outcome, &next) {
            (Ok(reply), Next::Send(command)) => {
                let level_ok = reply.ok();
                if level_ok {
                    tracing::info!(
                        "'{entity}': {} -> OK (agent {})",
                        command.name(),
                        reply.phase.as_deref().unwrap_or("phase unknown")
                    );
                } else {
                    tracing::warn!(
                        "'{entity}': {} -> {} {}",
                        command.name(),
                        reply.status,
                        reply.message
                    );
                }
                queue.on_reply(&entity, command, &reply, now);
                if let Some(error) = queue.error() {
                    tracing::error!("'{entity}': {error}");
                }
            }
            (Ok(reply), _) => {
                if queue.phase() != reply.phase.as_deref() {
                    tracing::info!(
                        "'{entity}': agent {} (goal held until it takes one)",
                        reply.phase.as_deref().unwrap_or("not registered")
                    );
                }
                queue.on_query(&reply, now);
            }
            (Err(e), _) => {
                tracing::warn!("Agent relay {relay}: {e:#}; reconnecting");
                connection = None;
                queue.on_link_down(&next, &format!("{e:#}"), relay, now);
            }
        }
    }
}

/// Role names csb gave agent-driven vehicles, kept across csb processes so a bridge that
/// died does not leave them for ever: their role is the entity name (that is how the vehicle
/// side finds them), so unlike other entities they carry no `csb_entity:` mark. On tmpfs,
/// beside the episode clock (`clock_store`).
pub fn ledger_path(host: &str, port: u16) -> PathBuf {
    let base = std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    base.join("carla-scenario-bridge")
        .join(format!("agent-vehicles-{host}-{port}"))
}

pub fn load_ledger(path: &Path) -> Vec<String> {
    std::fs::read_to_string(path)
        .map(|text| {
            text.lines()
                .map(str::trim)
                .filter(|l| !l.is_empty())
                .map(str::to_string)
                .collect()
        })
        .unwrap_or_default()
}

pub fn store_ledger<'a>(path: &Path, names: impl IntoIterator<Item = &'a String>) -> Result<()> {
    let mut names: Vec<&String> = names.into_iter().collect();
    names.sort();
    names.dedup();
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let text: String = names.iter().map(|n| format!("{n}\n")).collect();
    std::fs::write(path, text).wrap_err_with(|| format!("write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    const P: AgentPose = AgentPose {
        x: 230.0,
        y: -129.8,
        z: 0.0,
        qx: 0.0,
        qy: 0.0,
        qz: 1.0,
        qw: 0.0,
    };

    fn reply(status: &str, registered: bool, phase: Option<&str>) -> Reply {
        Reply {
            status: status.into(),
            message: String::new(),
            registered,
            phase: phase.map(str::to_string),
        }
    }

    fn goal() -> Command {
        Command::SetGoal {
            goal: P,
            waypoints: vec![],
        }
    }

    #[test]
    fn a_goal_request_maps_to_speed_then_goal() {
        let w = AgentPose { x: 1.0, ..P };
        assert_eq!(
            Command::from_goal_request(&[w, P], false, Some(8.0)),
            vec![
                Command::SetSpeedLimit(Some(8.0)),
                Command::SetGoal {
                    goal: P,
                    waypoints: vec![w]
                }
            ]
        );
        assert_eq!(
            Command::from_goal_request(&[], true, None),
            vec![Command::ClearGoal]
        );
        assert!(Command::from_goal_request(&[], false, None).is_empty());
    }

    #[test]
    fn commands_carry_the_protocol_args() {
        assert_eq!(Command::Teleported(P).args()["pose"]["qz"], 1.0);
        let args = goal().args();
        assert_eq!(args["goal"]["x"], 230.0);
        assert_eq!(args["waypoints"], json!([]));
        assert_eq!(Command::SetSpeedLimit(None).args(), json!({"mps": null}));
        assert_eq!(Command::Stop.name(), "stop");
    }

    #[test]
    fn teleported_goes_first_and_a_goal_waits_for_npc_logic_and_localization() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(Command::Teleported(P));
        q.push(goal());
        assert_eq!(q.next(now, false), Next::Send(Command::Teleported(P)));
        // The agent re-localizes: INITIALIZING (pushed before its reply).
        q.on_reply(
            "bg",
            &Command::Teleported(P),
            &reply("OK", true, Some("INITIALIZING")),
            now,
        );
        assert_eq!(q.next(now, false), Next::Idle, "NPC logic not started");
        assert_eq!(q.next(now, true), Next::Query, "agent still localizing");
        q.on_query(&reply("OK", true, Some("INITIALIZING")), now);
        assert_eq!(q.next(now, true), Next::Idle, "polls, does not spin");
        let later = now + POLL;
        q.on_query(&reply("OK", true, Some("IDLE")), later);
        assert_eq!(q.next(later + POLL, true), Next::Send(goal()));
        q.on_reply("bg", &goal(), &reply("OK", true, Some("PLANNING")), later);
        assert_eq!(q.next(later + POLL, true), Next::Idle);
        assert!(q.error().is_none());
    }

    #[test]
    fn a_teleported_without_an_agent_is_retried_not_failed() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(Command::Teleported(P));
        q.on_reply(
            "bg",
            &Command::Teleported(P),
            &reply("FAILED", false, None),
            now,
        );
        assert!(q.error().is_none());
        assert_eq!(q.next(now, true), Next::Idle);
        assert_eq!(
            q.next(now + RETRY, true),
            Next::Send(Command::Teleported(P))
        );
    }

    #[test]
    fn a_refused_goal_is_the_entitys_error() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(goal());
        q.on_query(&reply("OK", true, Some("IDLE")), now);
        let mut refused = reply("FAILED", true, Some("IDLE"));
        refused.message = "The planned route is empty".into();
        q.on_reply("bg_av_1", &goal(), &refused, now);
        let error = q.error().expect("an error");
        assert!(
            error.contains("set_goal with FAILED: The planned route is empty"),
            "{error}"
        );
        assert_eq!(q.next(now + POLL, true), Next::Idle, "nothing more is sent");
    }

    #[test]
    fn a_goal_that_raced_back_into_initializing_waits_again() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(goal());
        q.on_reply(
            "bg",
            &goal(),
            &reply("FAILED", true, Some("INITIALIZING")),
            now,
        );
        assert!(q.error().is_none());
        assert_eq!(q.next(now + POLL, true), Next::Query);
    }

    #[test]
    fn a_vanished_agent_fails_a_goal_with_the_entity_named() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(Command::SetSpeedLimit(Some(5.0)));
        q.on_reply(
            "bg_av_1",
            &Command::SetSpeedLimit(Some(5.0)),
            &reply("FAILED", false, None),
            now,
        );
        let error = q.error().expect("an error");
        assert!(
            error.contains("as 'bg_av_1'") && error.contains("entity:=bg_av_1"),
            "{error}"
        );
    }

    #[test]
    fn a_newer_goal_replaces_a_queued_one_and_despawn_sends_only_stop() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(goal());
        q.push(Command::ClearGoal);
        q.push(Command::SetSpeedLimit(Some(1.0)));
        q.push(Command::SetSpeedLimit(Some(2.0)));
        assert_eq!(
            q.queue,
            VecDeque::from(vec![Command::ClearGoal, Command::SetSpeedLimit(Some(2.0))])
        );
        q.error = Some("x".into());
        q.retire();
        assert_eq!(
            q.next(now, false),
            Next::Send(Command::Stop),
            "even after an error"
        );
        q.on_reply("bg", &Command::Stop, &reply("FAILED", false, None), now);
        assert!(q.done());
    }

    #[test]
    fn a_relay_outage_retries_teleports_and_fails_goals() {
        let now = Instant::now();
        let mut q = EntityQueue::default();
        q.push(Command::Teleported(P));
        let next = q.next(now, true);
        q.on_link_down(&next, "refused", "tcp://localhost:5560", now);
        assert!(q.error().is_none());
        q.queue.clear();
        q.push(Command::ClearGoal);
        let next = q.next(now + RETRY, true);
        q.on_link_down(&next, "refused", "tcp://localhost:5560", now);
        assert!(q
            .error()
            .unwrap()
            .contains("tcp://localhost:5560 is unreachable"));
    }

    #[test]
    fn relay_addresses_parse() {
        assert_eq!(
            parse_relay_address("tcp://localhost:5560").unwrap(),
            ("localhost".into(), 5560)
        );
        assert_eq!(parse_relay_address("sim:7").unwrap(), ("sim".into(), 7));
        assert_eq!(
            parse_relay_address("tcp://sim").unwrap(),
            ("sim".into(), 5560)
        );
        assert!(parse_relay_address("http://x:1").is_err());
        assert!(parse_relay_address("tcp://:1").is_err());
        assert!(parse_relay_address("tcp://x:port").is_err());
    }

    #[test]
    fn replies_parse_registration_and_phase() {
        let r = Reply::from_json(&json!({"type": "reply", "id": 1, "status": "OK",
            "message": "", "registered": true, "state": {"phase": "IDLE"}}))
        .unwrap();
        assert!(r.registered && r.phase.as_deref() == Some("IDLE"));
        let r = Reply::from_json(&json!({"type": "reply", "id": 1, "status": "FAILED",
            "message": "no agent", "registered": false}))
        .unwrap();
        assert!(!r.registered && r.phase.is_none() && r.message == "no agent");
    }

    #[test]
    fn the_ledger_round_trips() {
        let path = std::env::temp_dir()
            .join(format!("csb-ledger-test-{}", std::process::id()))
            .join("agents");
        assert!(load_ledger(&path).is_empty());
        let names = ["b".to_string(), "a".to_string(), "b".to_string()];
        store_ledger(&path, names.iter()).unwrap();
        assert_eq!(load_ledger(&path), ["a", "b"]);
        store_ledger(&path, std::iter::empty()).unwrap();
        assert!(load_ledger(&path).is_empty());
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    /// A fake relay speaking the commander side of the protocol, one connection.
    fn fake_relay(answers: Vec<Value>) -> (String, std::thread::JoinHandle<Vec<Value>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = format!("tcp://{}", listener.local_addr().unwrap());
        let handle = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut writer = stream;
            let mut seen = Vec::new();
            let mut answers = answers.into_iter();
            let mut line = String::new();
            while reader.read_line(&mut line).unwrap() > 0 {
                let message: Value = serde_json::from_str(&line).unwrap();
                line.clear();
                let mut answer = if message["type"] == "register" {
                    json!({"type": "registered", "entity": ""})
                } else {
                    let mut a = answers.next().unwrap_or(json!({"type": "reply",
                        "status": "OK", "message": ""}));
                    a["id"] = message["id"].clone();
                    a
                };
                seen.push(message);
                answer["v"] = json!(1);
                // A heartbeat first: the client must skip it.
                writer
                    .write_all(b"{\"v\":1,\"type\":\"heartbeat\"}\n")
                    .unwrap();
                writer.write_all(format!("{answer}\n").as_bytes()).unwrap();
            }
            seen
        });
        (address, handle)
    }

    #[test]
    fn the_connection_registers_as_a_commander_and_matches_replies() {
        let (address, relay) = fake_relay(vec![
            json!({"type": "reply", "status": "OK", "message": "", "registered": true,
                   "state": {"type": "state", "phase": "INITIALIZING"}}),
            json!({"type": "reply", "status": "FAILED", "message": "no agent registered",
                   "registered": false}),
        ]);
        let mut c = RelayConnection::connect(&address).unwrap();
        let r = c.command("bg_av_1", &Command::Teleported(P)).unwrap();
        assert!(r.ok() && r.phase.as_deref() == Some("INITIALIZING"));
        let r = c.query("nobody").unwrap();
        assert!(!r.registered);
        drop(c);
        let seen = relay.join().unwrap();
        assert_eq!(seen[0]["role"], "commander");
        assert_eq!(seen[0]["v"], 1);
        assert_eq!(seen[1]["type"], "command_for");
        assert_eq!(seen[1]["entity"], "bg_av_1");
        assert_eq!(seen[1]["command"], "teleported");
        assert_eq!(seen[1]["args"]["pose"]["x"], 230.0);
        assert_eq!(seen[2]["type"], "query");
        assert_ne!(seen[1]["id"], seen[2]["id"]);
    }

    #[test]
    fn check_agent_names_a_missing_agent_and_an_unreachable_relay() {
        let (address, _relay) = fake_relay(vec![json!({"type": "reply", "status": "FAILED",
            "message": "no agent registered for entity 'bg'", "registered": false})]);
        let mut link = AgentLink::start(&address);
        let e = link.check_agent("bg").unwrap_err().to_string();
        assert!(
            e.contains("no agent is registered") && e.contains("entity:=bg"),
            "{e}"
        );
        link.shutdown();

        // Nothing listens on a port we just released.
        let port = TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let mut link = AgentLink::start(&format!("tcp://127.0.0.1:{port}"));
        let e = format!("{:#}", link.check_agent("bg").unwrap_err());
        assert!(e.contains("unreachable"), "{e}");
    }
}
