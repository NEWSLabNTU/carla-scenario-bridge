//! The world protocol's server shell in Rust (roadmap 046): a [`World`] behind a REP socket.
//!
//! The Rust twin of the Python shell `csb_world_server`, for a simulator with a Rust or C++
//! client (the CARLA world server). The shell owns the protocol so a world does not have to:
//! the version check, the owner lease (one bridge at a time; `proto/csb_world.proto`,
//! "Ownership"), handles, request validation against the declared capabilities, and the
//! process's end. A [`World`] method returns a [`Refusal`] to say no; its text is what the
//! bridge passes on to SSv2.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use prost::Message;

use crate::proto::csb_world as wire;
use crate::world_client::{describe, is_gone, PROTOCOL_VERSION};

/// How long an owner may say nothing before another client may take the server over. The
/// bridge probes every few seconds between scenarios, so a live one never lapses.
pub const LEASE: Duration = Duration::from_secs(15);

/// A world's no: the reply's error. `fatal`: the world cannot go on, and the server ends
/// after this reply.
#[derive(Debug, Clone, PartialEq)]
pub struct Refusal {
    pub message: String,
    pub fatal: bool,
}

impl Refusal {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            fatal: false,
        }
    }

    pub fn fatal(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            fatal: true,
        }
    }
}

impl<E: std::fmt::Display> From<E> for Refusal {
    fn from(e: E) -> Self {
        Self::new(e.to_string())
    }
}

pub type WorldResult<T> = Result<T, Refusal>;

/// What a simulator implements to be driven over the world protocol. Handles are the
/// shell's: unique over the process's life, checked before a world sees them.
pub trait World {
    /// `carla`, `isaac`, ...: the map directory's `<name>/world.yaml` is this server's.
    fn simulator_name(&self) -> &str;
    fn simulator_version(&self) -> String;
    /// What the world offers beyond the core.
    fn capabilities(&self) -> Vec<wire::Capability>;
    /// The longest physics step the simulator takes, s; 0: it takes SSv2's step whole.
    fn max_physics_dt(&self) -> f64 {
        0.0
    }
    /// Whether a world can be loaded now, or why not.
    fn ready(&mut self) -> Result<(), String> {
        Ok(())
    }
    fn load_world(&mut self, request: &wire::LoadWorld) -> WorldResult<wire::LoadWorldReply>;
    fn spawn(&mut self, handle: u32, request: &wire::Spawn) -> WorldResult<wire::Body>;
    fn despawn(&mut self, handle: u32) -> WorldResult<()>;
    fn apply(&mut self, request: &wire::Apply) -> WorldResult<wire::ApplyReply>;
    fn step(&mut self, request: &wire::Step) -> WorldResult<wire::StepReply>;
    fn set_step_time(&mut self, _step_time: f64) -> WorldResult<()> {
        Ok(())
    }
    fn set_signals(&mut self, _request: &wire::SetSignals) -> WorldResult<()> {
        Err(Refusal::new(
            "traffic signals are not offered by this server",
        ))
    }
    fn set_goal(&mut self, _request: &wire::SetGoal) -> WorldResult<()> {
        Err(Refusal::new(
            "goals are not offered by this server (no AUTOPILOT)",
        ))
    }
    fn set_weather(&mut self, _request: &wire::SetWeather) -> WorldResult<wire::WeatherReply> {
        Err(Refusal::new("weather is not offered by this server"))
    }
    fn get_weather(&mut self) -> WorldResult<wire::WeatherReply> {
        Err(Refusal::new("weather is not offered by this server"))
    }
    /// A new owner: every entity of the last one goes (`handles`); the world stays.
    fn reset(&mut self, handles: &[u32]) -> WorldResult<()> {
        for h in handles {
            self.despawn(*h)?;
        }
        Ok(())
    }
    /// The owner leaves (`Release`): undo what was done for it. Default: as a reset.
    fn release(&mut self, handles: &[u32]) -> WorldResult<()> {
        self.reset(handles)
    }
    /// No request for a while (between scenarios): check the simulator. `Some(reason)`:
    /// it is gone, and the server ends (`serve` returns [`End::Lost`]).
    fn idle(&mut self) -> Option<String> {
        None
    }
    /// The server is stopping (not after a fatal end, which touches nothing).
    fn shutdown(&mut self) {}
}

/// Why [`serve`] returned.
#[derive(Debug, Clone, PartialEq)]
pub enum End {
    /// `Shutdown`, or the stop flag.
    Stopped,
    /// A world call failed fatally (a hung tick): the reason. The process should exit for a
    /// fresh one without touching the simulator again.
    Fatal(String),
    /// The idle check found the simulator gone: the reason.
    Lost(String),
}

pub struct Shell<W: World> {
    pub world: W,
    capabilities: BTreeSet<i32>,
    instance: String,
    owner: Option<wire::ClientId>,
    owner_seen: Instant,
    lease: Duration,
    next_handle: u32,
    /// Handle to entity name.
    entities: BTreeMap<u32, String>,
    stopped: bool,
    fatal: Option<String>,
    /// When the last request other than `Hello` came: the idle check runs only when the
    /// world has been left alone (a bridge's own idle probes are hellos).
    last_world_request: Instant,
    /// Test hook: whether a client is gone (default [`is_gone`]).
    gone: fn(&wire::ClientId) -> bool,
}

fn same(a: &wire::ClientId, b: &wire::ClientId) -> bool {
    (a.host.as_str(), a.pid, a.start_time) == (b.host.as_str(), b.pid, b.start_time)
}

fn ok(reply: Option<wire::reply::Reply>) -> wire::Reply {
    wire::Reply {
        ok: true,
        error: String::new(),
        reply,
        fatal: false,
    }
}

fn refused(r: Refusal) -> wire::Reply {
    wire::Reply {
        ok: false,
        error: r.message,
        reply: None,
        fatal: r.fatal,
    }
}

impl<W: World> Shell<W> {
    pub fn new(world: W) -> Self {
        let capabilities = world.capabilities().into_iter().map(|c| c as i32).collect();
        let instance = format!(
            "{:x}{:x}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or_default()
        );
        Self {
            world,
            capabilities,
            instance,
            owner: None,
            owner_seen: Instant::now(),
            lease: LEASE,
            next_handle: 1,
            entities: BTreeMap::new(),
            stopped: false,
            fatal: None,
            last_world_request: Instant::now(),
            gone: is_gone,
        }
    }

    pub fn with_lease(mut self, lease: Duration, gone: fn(&wire::ClientId) -> bool) -> Self {
        self.lease = lease;
        self.gone = gone;
        self
    }

    pub fn offers(&self, c: wire::Capability) -> bool {
        self.capabilities.contains(&(c as i32))
    }

    pub fn owner(&self) -> Option<&wire::ClientId> {
        self.owner.as_ref()
    }

    pub fn stopped(&self) -> bool {
        self.stopped
    }

    /// Why the world cannot go on, once a call failed fatally.
    pub fn fatal(&self) -> Option<&str> {
        self.fatal.as_deref()
    }

    /// One request's bytes to its reply's bytes. Never panics on the peer's input.
    pub fn handle(&mut self, data: &[u8]) -> Vec<u8> {
        let reply = match wire::Request::decode(data) {
            Err(e) => refused(Refusal::new(format!(
                "undecodable request ({} bytes): {e}",
                data.len()
            ))),
            Ok(request) => {
                let client = request.client.unwrap_or_default();
                match request.request {
                    None => refused(Refusal::new("empty request")),
                    Some(r) => match self.dispatch(r, &client) {
                        Ok(reply) => reply,
                        Err(r) => {
                            if r.fatal {
                                self.fatal = Some(r.message.clone());
                            }
                            refused(r)
                        }
                    },
                }
            }
        };
        reply.encode_to_vec()
    }

    fn check_owner(&mut self, what: &str, client: &wire::ClientId) -> WorldResult<()> {
        if let Some(reason) = &self.fatal {
            return Err(Refusal::fatal(reason.clone()));
        }
        let Some(owner) = &self.owner else {
            return Err(Refusal::new(format!(
                "{what} before hello: say hello first"
            )));
        };
        if !same(client, owner) {
            return Err(Refusal::new(format!(
                "{what} from {} refused: this world server is owned by {}",
                describe(client),
                describe(owner)
            )));
        }
        self.owner_seen = Instant::now();
        Ok(())
    }

    fn take(&mut self, client: &wire::ClientId, why: &str) -> WorldResult<()> {
        match &self.owner {
            Some(old) => tracing::info!(
                "World server owner {} replaced by {} ({why})",
                describe(old),
                describe(client)
            ),
            None => tracing::info!("World server owner: {} ({why})", describe(client)),
        }
        let handles: Vec<u32> = self.entities.keys().copied().collect();
        self.owner = Some(client.clone());
        self.owner_seen = Instant::now();
        if !handles.is_empty() {
            tracing::info!(
                "Removing the {} entit{} of the last owner",
                handles.len(),
                if handles.len() == 1 { "y" } else { "ies" }
            );
            self.entities.clear();
            self.world.reset(&handles)?;
        }
        Ok(())
    }

    fn hello(&mut self, req: wire::Hello, client: &wire::ClientId) -> WorldResult<wire::Reply> {
        if req.protocol_version != PROTOCOL_VERSION {
            return Err(Refusal::new(format!(
                "protocol version {} refused: this world server speaks {PROTOCOL_VERSION}",
                req.protocol_version
            )));
        }
        if !req.probe {
            match self.owner.clone() {
                None => self.take(client, "first hello")?,
                Some(o) if same(&o, client) => self.owner_seen = Instant::now(),
                Some(o) if (self.gone)(&o) => self.take(client, "the owner's process is gone")?,
                Some(_) if self.owner_seen.elapsed() > self.lease => {
                    let why = format!("the owner was silent for more than {:?}", self.lease);
                    self.take(client, &why)?
                }
                Some(o) => {
                    return Err(Refusal::new(format!(
                        "owned by {} (seen {:.0} s ago): one bridge drives a world server at a \
                         time",
                        describe(&o),
                        self.owner_seen.elapsed().as_secs_f64()
                    )))
                }
            }
        }
        let ready = self.world.ready();
        Ok(ok(Some(wire::reply::Reply::Hello(wire::HelloReply {
            protocol_version: PROTOCOL_VERSION,
            simulator_version: self.world.simulator_version(),
            simulator_name: self.world.simulator_name().to_string(),
            capabilities: self.capabilities.iter().copied().collect(),
            owner: self.owner.clone(),
            instance: self.instance.clone(),
            max_physics_dt: self.world.max_physics_dt(),
            ready: ready.is_ok(),
            not_ready_reason: ready.err().unwrap_or_default(),
        }))))
    }

    fn need(&self, c: wire::Capability, what: &str) -> WorldResult<()> {
        if self.offers(c) {
            Ok(())
        } else {
            Err(Refusal::new(format!("{what} not offered by this server")))
        }
    }

    fn known(&self, handle: u32) -> WorldResult<()> {
        if self.entities.contains_key(&handle) {
            Ok(())
        } else {
            Err(Refusal::new(format!("no entity with handle {handle}")))
        }
    }

    fn dispatch(
        &mut self,
        request: wire::request::Request,
        client: &wire::ClientId,
    ) -> WorldResult<wire::Reply> {
        use wire::reply::Reply as R;
        use wire::request::Request as Q;
        if let Q::Hello(h) = request {
            return self.hello(h, client);
        }
        let what = match &request {
            Q::Hello(_) => "hello",
            Q::LoadWorld(_) => "load_world",
            Q::Spawn(_) => "spawn",
            Q::Despawn(_) => "despawn",
            Q::Step(_) => "step",
            Q::Shutdown(_) => "shutdown",
            Q::SetSignals(_) => "set_signals",
            Q::Apply(_) => "apply",
            Q::SetStepTime(_) => "set_step_time",
            Q::SetGoal(_) => "set_goal",
            Q::SetWeather(_) => "set_weather",
            Q::GetWeather(_) => "get_weather",
            Q::Release(_) => "release",
        };
        self.check_owner(what, client)?;
        self.last_world_request = Instant::now();
        Ok(match request {
            Q::Hello(_) => unreachable!("answered above"),
            Q::LoadWorld(req) => {
                if let Err(why) = self.world.ready() {
                    return Err(Refusal::new(format!("not ready to load a world: {why}")));
                }
                let reply = self.world.load_world(&req)?;
                self.entities.clear();
                ok(Some(R::LoadWorld(reply)))
            }
            Q::Spawn(req) => {
                if self.entities.values().any(|n| *n == req.name) {
                    return Err(Refusal::new(format!(
                        "an entity named {:?} exists",
                        req.name
                    )));
                }
                let motion = wire::Motion::try_from(req.motion).unwrap_or(wire::Motion::Kinematic);
                match motion {
                    wire::Motion::Tracked
                        if !self.offers(wire::Capability::TrackedMotion)
                            && !self.offers(wire::Capability::TracksTargets) =>
                    {
                        return Err(Refusal::new(format!(
                            "{:?}: TRACKED bodies are not offered by this server",
                            req.name
                        )))
                    }
                    wire::Motion::Autopilot => {
                        self.need(wire::Capability::Autopilot, "AUTOPILOT bodies are")?
                    }
                    wire::Motion::External if req.kind != wire::Kind::Ego as i32 => self.need(
                        wire::Capability::AgentVehicles,
                        "externally driven vehicles other than the ego are",
                    )?,
                    _ => {}
                }
                let handle = self.next_handle;
                let body = self.world.spawn(handle, &req)?;
                self.next_handle += 1;
                self.entities.insert(handle, req.name.clone());
                ok(Some(R::Spawn(wire::SpawnReply {
                    handle,
                    body: Some(body),
                })))
            }
            Q::Despawn(req) => {
                self.known(req.handle)?;
                self.world.despawn(req.handle)?;
                self.entities.remove(&req.handle);
                ok(None)
            }
            Q::Apply(req) => {
                let unknown: Vec<u32> = req
                    .poses
                    .iter()
                    .map(|p| p.handle)
                    .filter(|h| !self.entities.contains_key(h))
                    .collect();
                if !unknown.is_empty() {
                    return Err(Refusal::new(format!(
                        "poses for unknown handle(s) {unknown:?}"
                    )));
                }
                ok(Some(R::Apply(self.world.apply(&req)?)))
            }
            Q::Step(req) => {
                if !req.poses.is_empty() {
                    return Err(Refusal::new(
                        "Step carries no poses since protocol 4: send them by Apply",
                    ));
                }
                ok(Some(R::Step(self.world.step(&req)?)))
            }
            Q::SetStepTime(req) => {
                self.world.set_step_time(req.step_time)?;
                ok(None)
            }
            Q::SetSignals(req) => {
                self.need(wire::Capability::Signals, "traffic signals are")?;
                self.world.set_signals(&req)?;
                ok(None)
            }
            Q::SetGoal(req) => {
                self.need(wire::Capability::Autopilot, "goals are")?;
                self.known(req.handle)?;
                self.world.set_goal(&req)?;
                ok(None)
            }
            Q::SetWeather(req) => {
                self.need(wire::Capability::Weather, "weather is")?;
                ok(Some(R::Weather(self.world.set_weather(&req)?)))
            }
            Q::GetWeather(_) => {
                self.need(wire::Capability::Weather, "weather is")?;
                ok(Some(R::Weather(self.world.get_weather()?)))
            }
            Q::Release(_) => {
                let handles: Vec<u32> = self.entities.keys().copied().collect();
                tracing::info!(
                    "World server released by {}",
                    self.owner.as_ref().map(describe).unwrap_or_default()
                );
                self.entities.clear();
                self.owner = None;
                self.world.release(&handles)?;
                ok(None)
            }
            Q::Shutdown(_) => {
                self.stopped = true;
                ok(None)
            }
        })
    }
}

/// The REP socket on `endpoint`, bound now. A server binds before its simulator's start, so
/// a second one on the port fails at once (`Address already in use`) and exits 1 -- which
/// its supervisor does not respawn.
pub fn bind(ctx: &zmq::Context, endpoint: &str) -> eyre::Result<zmq::Socket> {
    let socket = ctx.socket(zmq::REP)?;
    socket.set_linger(1000)?;
    socket
        .bind(endpoint)
        .map_err(|e| eyre::eyre!("cannot bind {endpoint}: {e} (another world server holds it)"))?;
    Ok(socket)
}

/// Answer requests on `socket` until `Shutdown`, `stop`, a fatal world call, or the idle
/// check finding the simulator gone. `idle_every`: how long without a request before
/// [`World::idle`] runs (and then again every so often).
pub fn serve<W: World>(
    socket: &zmq::Socket,
    shell: &mut Shell<W>,
    stop: &AtomicBool,
    idle_every: Duration,
) -> eyre::Result<End> {
    let mut last_idle = Instant::now();
    loop {
        if shell.stopped() || stop.load(Ordering::SeqCst) {
            return Ok(End::Stopped);
        }
        if let Some(reason) = shell.fatal() {
            return Ok(End::Fatal(reason.to_string()));
        }
        let ready = socket.poll(zmq::POLLIN, 100)? > 0;
        if !ready {
            if shell.last_world_request.elapsed() >= idle_every && last_idle.elapsed() >= idle_every
            {
                last_idle = Instant::now();
                if let Some(reason) = shell.world.idle() {
                    return Ok(End::Lost(reason));
                }
            }
            continue;
        }
        let request = socket.recv_bytes(0)?;
        let reply = shell.handle(&request);
        socket.send(reply, 0)?;
    }
}

/// Ask the kernel for SIGTERM when the parent dies (a supervisor killed with SIGKILL), and
/// exit 1 now when it already has. Linux only.
pub fn die_with_parent() {
    // SAFETY: getppid and prctl have no memory-safety preconditions.
    unsafe {
        let parent = libc::getppid();
        libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM);
        if parent == 1 || libc::getppid() != parent {
            eprintln!("parent process gone before start: exiting");
            libc::_exit(1);
        }
    }
}

#[cfg(test)]
mod tests;
