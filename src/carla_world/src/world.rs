//! [`CarlaWorld`]: CARLA as the world protocol's `World` (roadmap 046).
//!
//! The protocol's requests become calls on [`CarlaBackend`], the CARLA backend the bridge used
//! to link (roadmap 035), unchanged in what it does to CARLA: SSv2's two calls per frame are
//! the protocol's `Apply` and `Step`, so every CARLA call happens when it did. What the bridge
//! now does itself -- the agent link, the gap trace, contact reports -- is off here; the
//! backend hands up what they need instead (spawn bodies, read-backs, tracking samples and
//! flags, contacts, post-tick states).
//!
//! Frames: the protocol's world frame is the map frame less the map origin the bridge read
//! from `<map dir>/carla/world.yaml` (or `carla/map_origin.yaml`); the backend works in the
//! map frame and applies the same origin to reach CARLA's. Poses are translated by the origin
//! on the way in and out, nothing else.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::backend::carla::{self, CarlaBackend};
use crate::backend::{
    BackendError, Driver, EntityKind, EntityStatus, Environment, Episode, Goal, RoadCondition,
    SimBackend, SpawnRequest, StatusBatch,
};
use crate::config::{BridgeConfig, NpcMotion};
use crate::coordinate_conversion::euler_to_quaternion;
use crate::proto::geometry_msgs::{self, Pose};
use crate::proto::swb_world as wire;
use crate::proto::traffic_simulator_msgs::entity_subtype::Enum as EntitySubtype;
use crate::proto::traffic_simulator_msgs::{ActionStatus, BoundingBox};
use crate::weather::wire_form;
use swb_core::world_server::{Refusal, World, WorldResult};

/// How often a CARLA that is not there yet is tried again.
const CONNECT_EVERY: Duration = Duration::from_secs(5);

pub struct CarlaWorld {
    config: BridgeConfig,
    config_dir: PathBuf,
    backend: Option<CarlaBackend>,
    /// Why the last connection attempt failed, and when it was made.
    not_ready: String,
    last_attempt: Option<Instant>,
    /// The entities by handle: name, what they are and how they move.
    entities: BTreeMap<u32, Entity>,
    /// Handle by name.
    handles: BTreeMap<String, u32>,
    map_origin: [f64; 3],
}

#[derive(Debug, Clone)]
struct Entity {
    name: String,
    kind: wire::Kind,
    motion: wire::Motion,
}

fn v3(a: [f64; 3]) -> Option<wire::Vec3> {
    Some(wire::Vec3 {
        x: a[0],
        y: a[1],
        z: a[2],
    })
}

fn refusal(e: BackendError, backend: &CarlaBackend) -> Refusal {
    match backend.wedged() {
        Some(reason) => Refusal::fatal(format!("{e} ({reason})")),
        None => Refusal::new(e.to_string()),
    }
}

impl CarlaWorld {
    pub fn new(config: BridgeConfig, config_dir: PathBuf) -> Self {
        Self {
            config,
            config_dir,
            backend: None,
            not_ready: "not tried yet".into(),
            last_attempt: None,
            entities: BTreeMap::new(),
            handles: BTreeMap::new(),
            map_origin: [0.0; 3],
        }
    }

    pub fn backend_mut(&mut self) -> Option<&mut CarlaBackend> {
        self.backend.as_mut()
    }

    /// One attempt to reach CARLA, at most every [`CONNECT_EVERY`]. Builds the backend on
    /// success: the epoch marker, the configured weather (`before_serving`).
    pub fn try_connect(&mut self) {
        if self.backend.is_some()
            || self
                .last_attempt
                .is_some_and(|t| t.elapsed() < CONNECT_EVERY)
        {
            return;
        }
        self.last_attempt = Some(Instant::now());
        let (host, port) = (self.config.carla.host.clone(), self.config.carla.port);
        let client = match carla::connect_once(&host, port) {
            Ok(c) => c,
            Err(e) => {
                if e != self.not_ready {
                    tracing::warn!("{e}; trying again every {} s", CONNECT_EVERY.as_secs());
                }
                self.not_ready = e;
                return;
            }
        };
        let world = match client.world() {
            Ok(w) => w,
            Err(e) => {
                self.not_ready = format!("CARLA world: {e}");
                return;
            }
        };
        tracing::info!("CARLA world acquired");
        let mut backend = CarlaBackend::new(
            client,
            world,
            host,
            port,
            self.config_dir.clone(),
            self.config.clone(),
        );
        backend.before_serving();
        self.backend = Some(backend);
        self.not_ready.clear();
    }

    fn backend(&mut self) -> WorldResult<&mut CarlaBackend> {
        let why = self.not_ready.clone();
        self.backend
            .as_mut()
            .ok_or_else(|| Refusal::new(format!("not connected to CARLA: {why}")))
    }

    fn name(&self, handle: u32) -> WorldResult<String> {
        self.entities
            .get(&handle)
            .map(|e| e.name.clone())
            .ok_or_else(|| Refusal::new(format!("no entity with handle {handle}")))
    }

    /// A world-frame pose in the map frame the backend works in.
    fn to_map(&self, pose: &wire::Pose) -> Pose {
        let p = pose.position.unwrap_or_default();
        let q = pose.orientation.unwrap_or(wire::Quat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        });
        let o = self.map_origin;
        Pose {
            position: Some(geometry_msgs::Point {
                x: p.x + o[0],
                y: p.y + o[1],
                z: p.z + o[2],
            }),
            orientation: Some(geometry_msgs::Quaternion {
                x: q.x,
                y: q.y,
                z: q.z,
                w: q.w,
            }),
        }
    }

    /// A map-frame pose in the world frame.
    fn to_world(&self, pose: &Pose) -> wire::Pose {
        let p = pose.position.unwrap_or_default();
        let q = pose.orientation.unwrap_or_default();
        let o = self.map_origin;
        wire::Pose {
            position: v3([p.x - o[0], p.y - o[1], p.z - o[2]]),
            orientation: Some(wire::Quat {
                x: q.x,
                y: q.y,
                z: q.z,
                w: q.w,
            }),
            rpy: None,
        }
    }

    /// The contacts the backend's sensors reported since the last call, in protocol terms.
    fn contacts(&mut self) -> Vec<wire::Contact> {
        let Some(backend) = self.backend.as_mut() else {
            return Vec::new();
        };
        let raw = backend.take_contacts();
        let ego = self
            .entities
            .iter()
            .find(|(_, e)| e.kind == wire::Kind::Ego)
            .map(|(h, _)| *h);
        raw.into_iter()
            .filter_map(|(event, other_name)| {
                // The reporting body: the NPC whose own sensor saw it, else the ego.
                let handle = match &event.subject {
                    Some(name) => self.handles.get(name).copied(),
                    None => ego,
                }?;
                let other = other_name
                    .as_ref()
                    .and_then(|n| self.handles.get(n).copied())
                    .unwrap_or(0);
                Some(wire::Contact {
                    handle,
                    other,
                    other_path: String::new(),
                    impulse: event.impulse,
                    other_sim_id: event.other_id.unwrap_or(0),
                    other_type: event.other_type,
                    frame: event.carla_frame,
                    sim_time: event.sim_time,
                })
            })
            .collect()
    }
}

fn subtype_of(s: &str) -> EntitySubtype {
    EntitySubtype::from_str_name(&s.to_ascii_uppercase()).unwrap_or(EntitySubtype::Unknown)
}

fn colour_state(c: i32) -> ::carla::rpc::TrafficLightState {
    use ::carla::rpc::TrafficLightState as S;
    match wire::Colour::try_from(c).unwrap_or(wire::Colour::Off) {
        wire::Colour::Off => S::Off,
        wire::Colour::Red => S::Red,
        wire::Colour::Amber => S::Yellow,
        wire::Colour::Green => S::Green,
    }
}

impl World for CarlaWorld {
    fn simulator_name(&self) -> &str {
        "carla"
    }

    fn simulator_version(&self) -> String {
        format!("CARLA {}", carla::version::BUILT_FOR)
    }

    fn capabilities(&self) -> Vec<wire::Capability> {
        use wire::Capability as C;
        vec![
            C::Signals,
            C::Autopilot,
            C::AgentVehicles,
            C::TracksTargets,
            C::ReadsBack,
            C::Weather,
            C::Contacts,
            C::RoadCondition,
        ]
    }

    fn ready(&mut self) -> Result<(), String> {
        self.try_connect();
        match &self.backend {
            Some(_) => Ok(()),
            None => Err(format!(
                "connecting to CARLA at {}:{} ({})",
                self.config.carla.host, self.config.carla.port, self.not_ready
            )),
        }
    }

    fn load_world(&mut self, req: &wire::LoadWorld) -> WorldResult<wire::LoadWorldReply> {
        self.map_origin = req.map_origin.map_or([0.0; 3], |o| [o.x, o.y, o.z]);
        let backend = self.backend()?;
        // `carla/world.yaml` names the town when the map directory's name is not it.
        if !req.world.is_empty() {
            if let Some(dir) = Path::new(&req.map_dir).file_name().and_then(|n| n.to_str()) {
                backend.alias_town(dir, &req.world);
            }
        }
        let episode = Episode {
            step_time: req.step_time,
            realtime_factor: req.realtime_factor,
            lanelet2_map_path: req.lanelet2_map_path.clone(),
        };
        let result = backend.initialize(&episode);
        let ns = result.map_err(|e| refusal(e, backend))?;
        // CARLA's origin and the bridge's must agree, or every pose is off by the difference.
        let ours = backend.map_origin();
        if ours != self.map_origin {
            tracing::warn!(
                "The bridge's map origin {:?} is not CARLA's {ours:?}: poses are translated by \
                 the bridge's",
                self.map_origin
            );
        }
        self.entities.clear();
        self.handles.clear();
        Ok(wire::LoadWorldReply { sim_time_ns: ns })
    }

    fn spawn(&mut self, handle: u32, req: &wire::Spawn) -> WorldResult<wire::Body> {
        let kind = wire::Kind::try_from(req.kind).unwrap_or(wire::Kind::Unspecified);
        let motion = wire::Motion::try_from(req.motion).unwrap_or(wire::Motion::Kinematic);
        let pose = req.pose.as_ref().map(|p| self.to_map(p));
        let bounding_box = req.box_declared.then(|| BoundingBox {
            center: req.box_center.map(|c| geometry_msgs::Point {
                x: c.x,
                y: c.y,
                z: c.z,
            }),
            dimensions: req.box_dimensions.map(|d| geometry_msgs::Vector3 {
                x: d.x,
                y: d.y,
                z: d.z,
            }),
        });
        let entity_kind = match kind {
            wire::Kind::Ego | wire::Kind::Vehicle => EntityKind::Vehicle {
                driver: match (kind, motion) {
                    (wire::Kind::Ego, _) => Driver::Ego,
                    (_, wire::Motion::External) => Driver::Agent,
                    (_, wire::Motion::Autopilot) => Driver::SimulatorAutopilot,
                    _ => Driver::Scenario,
                },
                subtype: subtype_of(&req.subtype),
                axles: req.axles.map(|a| (a.front, a.rear)),
                performance: req
                    .performance
                    .map(|p| (p.max_acceleration, p.max_deceleration)),
            },
            wire::Kind::Pedestrian => EntityKind::Pedestrian,
            _ => EntityKind::MiscObject,
        };
        let backend = self.backend()?;
        if kind == wire::Kind::Ego
            && !req.role_name.is_empty()
            && req.role_name != backend.ego_role_name()
        {
            return Err(Refusal::new(format!(
                "the ego's role name is '{}' on the bridge and '{}' here (carla_world.yaml \
                 ego.role_name): acb attaches by it, so they must agree",
                req.role_name,
                backend.ego_role_name()
            )));
        }
        // Tracked or posed: the bridge asks per entity (its `npc_motion`).
        backend.set_npc_motion(if motion == wire::Motion::Tracked {
            NpcMotion::Tracked
        } else {
            NpcMotion::Kinematic
        });
        let request = SpawnRequest {
            name: req.name.clone(),
            asset_key: req.asset_key.clone(),
            pose,
            bounding_box,
            kind: entity_kind,
        };
        let result = backend.spawn(&request);
        result.map_err(|e| refusal(e, backend))?;
        let body = backend.take_spawn_body();
        self.entities.insert(
            handle,
            Entity {
                name: req.name.clone(),
                kind,
                motion,
            },
        );
        self.handles.insert(req.name.clone(), handle);
        Ok(match body {
            Some(b) => wire::Body {
                sim_id: b.actor_id,
                model: b.blueprint,
                box_location: b.blueprint_box.as_ref().and_then(|x| v3(x.location)),
                box_extent: b.blueprint_box.as_ref().and_then(|x| v3(x.extent)),
                has_box: b.blueprint_box.is_some(),
                origin_offset: v3([b.origin_offset[0], b.origin_offset[1], 0.0]),
                origin_yaw: b.origin_offset[2],
                declared_center: v3(b.declared_center),
                declared_dimensions: v3(b.declared_dimensions),
            },
            None => wire::Body::default(),
        })
    }

    fn despawn(&mut self, handle: u32) -> WorldResult<()> {
        let name = self.name(handle)?;
        let backend = self.backend()?;
        let result = backend.despawn(&name);
        result.map_err(|e| refusal(e, backend))?;
        self.entities.remove(&handle);
        self.handles.remove(&name);
        Ok(())
    }

    fn apply(&mut self, req: &wire::Apply) -> WorldResult<wire::ApplyReply> {
        let mut entities = Vec::with_capacity(req.poses.len());
        for p in &req.poses {
            let name = self.name(p.handle)?;
            let action_status = p.ssv2_twist.map(|t| {
                let l = t.linear.unwrap_or_default();
                let a = t.angular.unwrap_or_default();
                ActionStatus {
                    twist: Some(geometry_msgs::Twist {
                        linear: Some(geometry_msgs::Vector3 {
                            x: l.x,
                            y: l.y,
                            z: l.z,
                        }),
                        angular: Some(geometry_msgs::Vector3 {
                            x: a.x,
                            y: a.y,
                            z: a.z,
                        }),
                    }),
                    ..Default::default()
                }
            });
            entities.push(EntityStatus {
                name,
                action_status,
                pose: p.pose.as_ref().map(|q| self.to_map(q)),
            });
        }
        let overwrite = req.overwrite_ego_status;
        let backend = self.backend()?;
        let outcome = backend.update_entity_status(StatusBatch {
            entities,
            npc_logic_started: req.npc_logic_started,
            overwrite_ego_status: overwrite,
        });
        if let Some(reason) = backend.wedged() {
            return Err(Refusal::fatal(reason.to_string()));
        }
        let samples = backend.take_tracking_samples();
        let mut reply = wire::ApplyReply {
            failure: outcome.failure.unwrap_or_default(),
            ..Default::default()
        };
        // Read back what CARLA moves: the ego (unless SSv2 drives it), agent and autopilot
        // vehicles.
        for status in &outcome.entities {
            let Some(&handle) = self.handles.get(&status.name) else {
                continue;
            };
            let entity = &self.entities[&handle];
            let read_back = match entity.kind {
                wire::Kind::Ego => !overwrite,
                _ => matches!(
                    entity.motion,
                    wire::Motion::External | wire::Motion::Autopilot
                ),
            };
            if !read_back {
                continue;
            }
            let a = status.action_status.clone().unwrap_or_default();
            let vec = |v: Option<geometry_msgs::Vector3>| {
                let v = v.unwrap_or_default();
                v3([v.x, v.y, v.z])
            };
            reply.read_back.push(wire::ReadBack {
                handle,
                pose: status.pose.as_ref().map(|p| self.to_world(p)),
                twist: a.twist.map(|t| wire::Twist {
                    linear: vec(t.linear),
                    angular: vec(t.angular),
                }),
                accel: a.accel.and_then(|x| vec(x.linear)),
                linear_jerk: a.linear_jerk,
            });
        }
        // Tracked bodies: this frame's flags and tracking records.
        let backend = self.backend.as_ref().expect("connected above");
        let mut by_name: BTreeMap<String, wire::TrackingSample> = BTreeMap::new();
        for (handle, e) in &self.entities {
            if e.motion != wire::Motion::Tracked {
                continue;
            }
            if let Some(flags) = backend.tracking_flags(&e.name) {
                by_name.insert(
                    e.name.clone(),
                    wire::TrackingSample {
                        handle: *handle,
                        held: flags.held,
                        kinematic: flags.kinematic,
                        ..Default::default()
                    },
                );
            }
        }
        for (name, sample) in samples {
            let Some(&handle) = self.handles.get(&name) else {
                continue;
            };
            let s = by_name.entry(name).or_insert(wire::TrackingSample {
                handle,
                ..Default::default()
            });
            use carla::collision_monitor::TrackingSample as T;
            match sample {
                T::Measured {
                    error,
                    yaw_error,
                    snapped,
                } => {
                    s.measured = true;
                    s.error = error;
                    s.has_yaw_error = yaw_error.is_some();
                    s.yaw_error = yaw_error.unwrap_or_default();
                    s.snapped = snapped;
                }
                T::Missed => s.missed = true,
                T::Fallback => s.fallback = true,
            }
        }
        reply.tracking = by_name.into_values().collect();
        reply.contacts = self.contacts();
        Ok(reply)
    }

    fn step(&mut self, req: &wire::Step) -> WorldResult<wire::StepReply> {
        let backend = self.backend()?;
        let result = backend.update_frame(req.current_simulation_time);
        let tick = backend.take_tick_time();
        let ns = result.map_err(|e| refusal(e, backend))?;
        let frame = backend.last_tick_frame().unwrap_or(0);
        let states = if req.want_states {
            backend.actor_states()
        } else {
            Vec::new()
        };
        let o = self.map_origin;
        let states = states
            .into_iter()
            .filter_map(|(name, p, v)| {
                let handle = *self.handles.get(&name)?;
                let q = euler_to_quaternion(p[3], p[4], p[5]);
                Some(wire::EntityState {
                    handle,
                    pose: Some(wire::Pose {
                        position: v3([p[0] - o[0], p[1] - o[1], p[2] - o[2]]),
                        orientation: Some(wire::Quat {
                            x: q.0,
                            y: q.1,
                            z: q.2,
                            w: q.3,
                        }),
                        rpy: v3([p[3], p[4], p[5]]),
                    }),
                    twist: Some(wire::Twist {
                        linear: v3(v),
                        angular: None,
                    }),
                })
            })
            .collect();
        let contacts = self.contacts();
        Ok(wire::StepReply {
            sim_time: ns as f64 / 1e9,
            frame,
            states,
            contacts,
            sim_time_ns: ns,
            tick_seconds: tick.as_secs_f64(),
        })
    }

    fn set_step_time(&mut self, step_time: f64) -> WorldResult<()> {
        let backend = self.backend()?;
        let result = backend.update_step_time(step_time);
        result.map_err(|e| refusal(e, backend))
    }

    fn set_signals(&mut self, req: &wire::SetSignals) -> WorldResult<()> {
        let states: Vec<(String, ::carla::rpc::TrafficLightState)> = req
            .states
            .iter()
            .map(|s| (s.opendrive_id.clone(), colour_state(s.colour)))
            .collect();
        let backend = self.backend()?;
        let result = backend.set_signal_states(&states);
        result.map_err(|e| refusal(e, backend))
    }

    fn set_goal(&mut self, req: &wire::SetGoal) -> WorldResult<()> {
        let name = self.name(req.handle)?;
        let goal = Goal {
            waypoints: req.waypoints.iter().map(|p| self.to_map(p)).collect(),
            clear: req.clear,
            target_speed: req.has_target_speed.then_some(req.target_speed),
        };
        let backend = self.backend()?;
        let result = backend.update_entity_goal(&name, &goal);
        result.map_err(|e| refusal(e, backend))
    }

    fn set_road_condition(&mut self, req: &wire::RoadCondition) -> WorldResult<()> {
        let env = Environment {
            road_condition: Some(RoadCondition {
                friction_scale_factor: req.friction_scale_factor,
                wetness: req.wetness.clone(),
                properties: req.properties.clone().into_iter().collect(),
            }),
        };
        let backend = self.backend()?;
        let result = backend.update_environment(&env);
        result.map_err(|e| refusal(e, backend))
    }

    fn set_weather(&mut self, req: &wire::SetWeather) -> WorldResult<wire::WeatherReply> {
        let request = wire_form::request_of(&req.weather.clone().unwrap_or_default())
            .map_err(Refusal::new)?;
        let backend = self.backend()?;
        Ok(wire_form::reply(&backend.set_weather(request)))
    }

    fn get_weather(&mut self) -> WorldResult<wire::WeatherReply> {
        let backend = self.backend()?;
        Ok(wire_form::reply(&SimBackend::get_weather(backend)))
    }

    fn reset(&mut self, handles: &[u32]) -> WorldResult<()> {
        for h in handles {
            if let Err(e) = self.despawn(*h) {
                tracing::warn!("Reset: {}", e.message);
            }
        }
        Ok(())
    }

    fn release(&mut self, _handles: &[u32]) -> WorldResult<()> {
        // Everything this server did to CARLA for the owner, undone: actors, the lights,
        // synchronous mode, the Traffic Manager.
        if let Some(backend) = self.backend.as_mut() {
            backend.shutdown();
        }
        self.entities.clear();
        self.handles.clear();
        Ok(())
    }

    fn idle(&mut self) -> Option<String> {
        match self.backend.as_mut() {
            None => {
                self.try_connect();
                None
            }
            Some(backend) => {
                backend.probe_connection_idle();
                backend.server_lost().map(str::to_string)
            }
        }
    }

    fn shutdown(&mut self) {
        if let Some(backend) = self.backend.as_mut() {
            backend.shutdown();
        }
    }
}
