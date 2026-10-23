//! `RemoteBackend` against an in-process world server: a REP socket on a thread that keeps
//! poses exactly, steps a clock, and reports a contact while the ego and another body are
//! within 1 m. Two stand-ins: one shaped like the Isaac server (the bridge tracks, the
//! bridge reads back), one like the CARLA server (`TRACKS_TARGETS`, `READS_BACK`).

use super::*;
use crate::proto::geometry_msgs::{Point, Quaternion};
use prost::Message;
use std::collections::HashMap;

/// What the stand-in server does with a request; `None` never answers.
type Behaviour = Box<dyn FnMut(&wire::Request) -> Option<wire::Reply> + Send>;

fn ok(reply: Option<wire::reply::Reply>) -> wire::Reply {
    wire::Reply {
        ok: true,
        error: String::new(),
        reply,
        fatal: false,
    }
}

fn refused(error: &str, fatal: bool) -> wire::Reply {
    wire::Reply {
        ok: false,
        error: error.into(),
        reply: None,
        fatal,
    }
}

const ISAAC_LIKE: &[wire::Capability] = &[
    wire::Capability::Signals,
    wire::Capability::TrackedMotion,
    wire::Capability::Contacts,
    wire::Capability::RoadCondition,
    wire::Capability::Environment,
];

const CARLA_LIKE: &[wire::Capability] = &[
    wire::Capability::Signals,
    wire::Capability::TracksTargets,
    wire::Capability::ReadsBack,
    wire::Capability::Autopilot,
    wire::Capability::AgentVehicles,
    wire::Capability::Weather,
    wire::Capability::Contacts,
    wire::Capability::RoadCondition,
    wire::Capability::Environment,
];

/// A server that behaves as the real ones do, as far as the backend can tell, with
/// `caps`. Under `READS_BACK` it reads the ego back at x + 0.5 and reports a sample for
/// every server-tracked body.
fn honest_with(caps: &'static [wire::Capability], instance: &'static str) -> Behaviour {
    let mut next = 1u32;
    // handle -> (pose, ego)
    let mut bodies: HashMap<u32, (wire::Pose, bool)> = HashMap::new();
    let mut velocity: HashMap<u32, [f64; 3]> = HashMap::new();
    let mut staged: Vec<wire::EntityPose> = Vec::new();
    let mut server_tracked: Vec<u32> = Vec::new();
    let mut time = 0.0;
    let mut frame = 0u64;
    let reads_back = caps.contains(&wire::Capability::ReadsBack);
    Box::new(move |req| {
        use wire::request::Request as R;
        Some(match req.request.clone()? {
            R::Hello(h) => ok(Some(wire::reply::Reply::Hello(wire::HelloReply {
                protocol_version: h.protocol_version,
                simulator_version: "stand-in 1".into(),
                simulator_name: "stand-in".into(),
                capabilities: caps.iter().map(|c| *c as i32).collect(),
                owner: req.client.clone(),
                instance: instance.into(),
                max_physics_dt: if reads_back { 0.0 } else { 1.0 / 60.0 },
                ready: true,
                not_ready_reason: String::new(),
            }))),
            R::LoadWorld(_) => {
                bodies.clear();
                ok(Some(wire::reply::Reply::LoadWorld(wire::LoadWorldReply {
                    sim_time_ns: (time * 1e9_f64).round() as i64,
                })))
            }
            R::Spawn(s) => {
                let h = next;
                next += 1;
                bodies.insert(h, (s.pose.unwrap(), s.kind == wire::Kind::Ego as i32));
                if s.motion == wire::Motion::Tracked as i32 {
                    if reads_back {
                        server_tracked.push(h);
                    } else {
                        velocity.insert(h, [0.0; 3]);
                    }
                }
                ok(Some(wire::reply::Reply::Spawn(wire::SpawnReply {
                    handle: h,
                    body: Some(wire::Body {
                        sim_id: if reads_back { 1000 + h } else { 0 },
                        model: "stand-in:box".into(),
                        ..Default::default()
                    }),
                })))
            }
            R::Despawn(d) => {
                bodies.remove(&d.handle);
                velocity.remove(&d.handle);
                ok(None)
            }
            R::Apply(a) => {
                staged = a.poses.clone();
                let mut reply = wire::ApplyReply::default();
                if reads_back {
                    for (h, (p, ego)) in &bodies {
                        if *ego {
                            let mut p = *p;
                            p.position.as_mut().unwrap().x += 0.5;
                            reply.read_back.push(wire::ReadBack {
                                handle: *h,
                                pose: Some(p),
                                twist: Some(wire::Twist {
                                    linear: v3([3.0, 0.0, 0.0]),
                                    angular: v3([0.0; 3]),
                                }),
                                accel: v3([0.25, 0.0, 0.0]),
                                linear_jerk: 0.5,
                            });
                        }
                    }
                    for p in &a.poses {
                        if server_tracked.contains(&p.handle) {
                            reply.tracking.push(wire::TrackingSample {
                                handle: p.handle,
                                held: true,
                                measured: true,
                                error: 0.125,
                                has_yaw_error: true,
                                yaw_error: 1.5,
                                ..Default::default()
                            });
                        }
                    }
                }
                ok(Some(wire::reply::Reply::Apply(reply)))
            }
            R::Step(s) => {
                let span = if reads_back {
                    0.05
                } else {
                    s.substeps as f64 * s.dt
                };
                for p in std::mem::take(&mut staged) {
                    if p.kinematic {
                        velocity.remove(&p.handle);
                    }
                    let tracked = velocity.contains_key(&p.handle);
                    let Some(body) = bodies.get_mut(&p.handle) else {
                        continue;
                    };
                    if let Some(pose) = p.pose {
                        if !tracked || p.teleport {
                            body.0 = pose;
                        }
                    }
                    if let (true, Some(t)) = (tracked, p.twist) {
                        // A tracked body moves at the velocity it is given, exactly.
                        let v = t.linear.unwrap();
                        let pos = body.0.position.as_mut().unwrap();
                        pos.x += v.x * span;
                        pos.y += v.y * span;
                        velocity.insert(p.handle, [v.x, v.y, v.z]);
                    }
                }
                time += span;
                frame += 1;
                let x = |p: &wire::Pose| p.position.unwrap().x;
                let mut contacts = Vec::new();
                for (h, (p, ego)) in &bodies {
                    for (o, (q, _)) in &bodies {
                        if *ego && h != o && (x(p) - x(q)).abs() < 1.0 {
                            contacts.push(wire::Contact {
                                handle: *h,
                                other: *o,
                                other_path: format!("/World/entities/e{o}"),
                                impulse: 1.0,
                                ..Default::default()
                            });
                        }
                    }
                }
                ok(Some(wire::reply::Reply::Step(wire::StepReply {
                    sim_time: time,
                    frame,
                    states: bodies
                        .iter()
                        .map(|(h, (p, _))| wire::EntityState {
                            handle: *h,
                            pose: Some(*p),
                            twist: velocity.get(h).map(|v| wire::Twist {
                                linear: v3(*v),
                                angular: v3([0.0; 3]),
                            }),
                        })
                        .collect(),
                    contacts,
                    sim_time_ns: 0,
                    tick_seconds: 0.0005,
                })))
            }
            R::SetGoal(_)
            | R::SetStepTime(_)
            | R::Shutdown(_)
            | R::SetSignals(_)
            | R::SetRoadCondition(_)
            | R::SetEnvironment(_)
            | R::Release(_) => ok(None),
            R::SetWeather(_) | R::GetWeather(_) => {
                let (_, w) = crate::weather::preset("ClearNoon").unwrap();
                ok(Some(wire::reply::Reply::Weather(wire_form::reply(
                    &WeatherReply::ok(w, "applied"),
                ))))
            }
        })
    })
}

fn honest() -> Behaviour {
    honest_with(ISAAC_LIKE, "one")
}

/// Serve `behaviour` on a port the OS picks; the endpoint.
fn server(mut behaviour: Behaviour) -> String {
    let ctx = zmq::Context::new();
    let socket = ctx.socket(zmq::REP).unwrap();
    socket.bind("tcp://127.0.0.1:*").unwrap();
    let endpoint = socket.get_last_endpoint().unwrap().unwrap();
    std::thread::spawn(move || {
        let _ctx = ctx;
        loop {
            let Ok(bytes) = socket.recv_bytes(0) else {
                return;
            };
            let req = wire::Request::decode(bytes.as_slice()).unwrap();
            match behaviour(&req) {
                Some(reply) => socket.send(reply.encode_to_vec(), 0).unwrap(),
                // Never answer: the REP socket cannot take another request either.
                None => loop {
                    std::thread::park();
                },
            }
        }
    });
    endpoint
}

/// A backend whose NPCs are kinematic, posed by SSv2.
fn backend(endpoint: &str) -> RemoteBackend {
    backend_with(endpoint, NpcMotion::Kinematic)
}

fn backend_with(endpoint: &str, motion: NpcMotion) -> RemoteBackend {
    let mut config = BridgeConfig {
        npc_motion: motion,
        ..Default::default()
    };
    config.world.endpoint = endpoint.to_string();
    config.world.request_timeout_seconds = 1;
    config.contact_report_dir = Some(contact_report::OFF.to_string());
    let shutdown = AtomicBool::new(false);
    connect(config, &shutdown).unwrap().expect("connected")
}

fn pose(x: f64, y: f64) -> Pose {
    Pose {
        position: Some(Point { x, y, z: 0.0 }),
        orientation: Some(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
    }
}

fn vehicle(name: &str, driver: Driver, x: f64) -> SpawnRequest {
    SpawnRequest {
        name: name.into(),
        asset_key: String::new(),
        pose: Some(pose(x, 0.0)),
        bounding_box: None,
        kind: EntityKind::Vehicle {
            driver,
            subtype: crate::proto::traffic_simulator_msgs::entity_subtype::Enum::Car,
            axles: None,
            performance: None,
        },
    }
}

fn episode() -> Episode {
    Episode {
        step_time: 0.05,
        realtime_factor: 1.0,
        lanelet2_map_path: "/nonexistent/Town01/lanelet2_map.osm".into(),
    }
}

fn status(name: &str, x: f64) -> EntityStatus {
    EntityStatus {
        name: name.into(),
        action_status: None,
        pose: Some(pose(x, 0.0)),
    }
}

fn batch(entities: Vec<EntityStatus>) -> StatusBatch {
    StatusBatch {
        entities,
        npc_logic_started: true,
        overwrite_ego_status: false,
    }
}

#[test]
fn a_frame_carries_the_poses_and_reads_the_ego_back() {
    let mut b = backend(&server(honest()));
    assert_eq!(b.server.name, "stand-in");
    assert_eq!(b.initialize(&episode()), Ok(0));
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, -10.0)).unwrap();
    assert_eq!(b.entity_count(), 2);

    // The NPC's pose goes out with Apply; the ego, not driven by SSv2, is read back.
    for frame in 1..=3 {
        let out = b.update_entity_status(batch(vec![
            status("ego", 99.0),
            status("npc", -10.0 + frame as f64),
        ]));
        assert_eq!(out.failure, None);
        let ego = out.entities.iter().find(|e| e.name == "ego").unwrap();
        assert_eq!(ego.pose.unwrap().position.unwrap().x, 0.0, "the ego stands");
        // The ego is listed with no drive: the simulator moves it.
        let sent = &b.sent[&b.entities["ego"].handle];
        assert!(sent.pose.is_none() && sent.twist.is_none());
        let t = b.update_frame(0.05 * frame as f64).unwrap();
        assert_eq!(t, 50_000_000 * frame);
    }
    assert_eq!(b.entities["npc"].pose.position.unwrap().x, -7.0);
    // 1 m/s along x, 20 frames a second: the NPC's motion is differenced from its poses.
    assert!((b.entities["npc"].motion.body[0] - 20.0).abs() < 1e-9);
    // The server's own stepping time (0.5 ms a step here), never more than the round trip.
    let tick = b.take_tick_time().as_secs_f64();
    assert!(tick > 0.0 && tick <= 0.0015 + 1e-9, "{tick}");

    // SSv2 drives the ego (a trajectory): its pose goes out too, and comes back from the
    // server.
    b.update_entity_status(StatusBatch {
        entities: vec![status("ego", 2.0)],
        npc_logic_started: true,
        overwrite_ego_status: true,
    });
    b.update_frame(0.2).unwrap();
    assert_eq!(b.entities["ego"].pose.position.unwrap().x, 2.0);
}

#[test]
fn a_server_that_reads_back_and_tracks_is_believed() {
    let mut b = backend_with(&server(honest_with(CARLA_LIKE, "one")), NpcMotion::Tracked);
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, -10.0)).unwrap();
    assert_eq!(b.entities["npc"].mover, Mover::ServerTracked);
    assert!(b.entities["npc"].tracked.is_none(), "no bridge law");
    // One step of the whole frame: the server splits it.
    assert_eq!((b.substeps, b.dt), (1, 0.05));
    let out = b.update_entity_status(batch(vec![
        status("ego", 99.0),
        EntityStatus {
            action_status: Some(ActionStatus {
                twist: Some(geometry_msgs::Twist {
                    linear: Some(geometry_msgs::Vector3 {
                        x: 4.0,
                        y: 0.0,
                        z: 0.0,
                    }),
                    angular: None,
                }),
                ..Default::default()
            }),
            ..status("npc", -9.0)
        },
    ]));
    assert_eq!(out.failure, None);
    // The ego as the server read it: its pose, twist, acceleration and jerk.
    let ego = out.entities.iter().find(|e| e.name == "ego").unwrap();
    assert_eq!(ego.pose.unwrap().position.unwrap().x, 0.5);
    let a = ego.action_status.as_ref().unwrap();
    assert_eq!(a.twist.unwrap().linear.unwrap().x, 3.0);
    assert_eq!(a.accel.unwrap().linear.unwrap().x, 0.25);
    assert_eq!(a.linear_jerk, 0.5);
    // The NPC: SSv2's pose and twist go to the server, which tracks it; SSv2's echoed.
    let npc = &b.sent[&b.entities["npc"].handle];
    assert_eq!(npc.pose.unwrap().position.unwrap().x, -9.0);
    assert_eq!(npc.ssv2_twist.unwrap().linear.unwrap().x, 4.0);
    assert!(npc.twist.is_none());
    let echoed = out.entities.iter().find(|e| e.name == "npc").unwrap();
    assert_eq!(echoed.pose.unwrap().position.unwrap().x, -9.0);
    // The server's tracking sample is counted.
    let s = b.entities["npc"].server_stats.as_ref().unwrap().summary();
    assert_eq!(s.frames, 1);
    assert_eq!(s.error_max, 0.125);
    // The simulator's own id names it.
    assert_eq!(b.entities["npc"].sim_id, 1002);
}

#[test]
fn the_egos_contacts_are_counted_per_other_entity() {
    let mut b = backend(&server(honest()));
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, -3.0)).unwrap();
    for x in [-2.0, -0.5, 0.0, 0.5, 2.0] {
        b.update_entity_status(batch(vec![status("npc", x)]));
        b.update_frame(0.0).unwrap();
    }
    let run = b.ego_run.as_ref().unwrap();
    assert_eq!(run.contacts.len(), 1);
    let c = &run.contacts[0].2;
    assert_eq!(c.name.as_deref(), Some("npc"));
    assert_eq!(c.other_type, "vehicle");
    assert_eq!((c.count, c.first_ssv2_frame), (3, 2));
}

#[test]
fn what_the_server_does_not_offer_is_an_honest_failure() {
    let mut b = backend(&server(honest()));
    b.initialize(&episode()).unwrap();
    for (driver, needs) in [
        (Driver::SimulatorAutopilot, "AUTOPILOT"),
        (Driver::Agent, "AGENT_VEHICLES"),
    ] {
        let e = b.spawn(&vehicle("bg", driver, 0.0)).unwrap_err();
        assert!(matches!(e, BackendError::Unsupported(_)), "{e:?}");
        assert!(e.to_string().contains(needs), "{e}");
    }
    assert_eq!(b.update_traffic_lights(&[]), Ok(()));
    // A signal the map's table does not have is warned about, not a failure.
    let light = TrafficSignal {
        id: 43763,
        ..Default::default()
    };
    assert_eq!(b.update_traffic_lights(&[light]), Ok(()));
    assert!(matches!(
        b.attach_sensor(Sensor::Lidar),
        Err(BackendError::Unsupported(_))
    ));
    assert!(!b.get_weather().success);
    assert!(b
        .set_weather(crate::weather::decode_request("ClearNoon", false, [0.0; 9]).unwrap())
        .message
        .contains("no weather"));
    // Unknown names fail, by name.
    let out = b.update_entity_status(batch(vec![status("ghost", 0.0)]));
    assert!(out.failure.unwrap().contains("'ghost'"));
    assert!(b
        .despawn("ghost")
        .unwrap_err()
        .to_string()
        .contains("'ghost' not found"));
    // A goal for an entity SSv2 drives.
    b.spawn(&vehicle("npc", Driver::Scenario, 0.0)).unwrap();
    let goal = Goal {
        waypoints: vec![pose(10.0, 0.0)],
        clear: false,
        target_speed: None,
    };
    let e = b.update_entity_goal("npc", &goal).unwrap_err();
    assert!(e.to_string().contains("not driven by the simulator"), "{e}");
    // One ego.
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    let e = b.spawn(&vehicle("ego2", Driver::Ego, 5.0)).unwrap_err();
    assert!(e.to_string().contains("ego 'ego' already exists"), "{e}");
}

fn road(f: f64, wetness: &str, props: &[(&str, &str)]) -> Environment {
    Environment {
        road_condition: Some(crate::backend::RoadCondition {
            friction_scale_factor: f,
            wetness: wetness.into(),
            properties: props
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        }),
        ..Default::default()
    }
}

#[test]
fn the_road_condition_goes_to_a_server_offering_it() {
    let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let log = seen.clone();
    let mut inner = honest();
    let mut b = backend(&server(Box::new(move |req| {
        if let Some(wire::request::Request::SetRoadCondition(r)) = &req.request {
            let rc = r.road_condition.clone().unwrap();
            let unknown = rc.properties.contains_key("swb.zone.nowhere.class");
            log.lock().unwrap().push(rc);
            if unknown {
                return Some(refused("zone 'nowhere' is not in the map", false));
            }
        }
        inner(req)
    })));
    b.initialize(&episode()).unwrap();
    let env = road(
        0.8,
        "lowFlooded",
        &[("swb.zone.ford.water_depth", "0.12"), ("esmini.thing", "1")],
    );
    assert_eq!(b.update_environment(&env), Ok(()));
    // Only our properties go to the server.
    let sent = seen.lock().unwrap().last().cloned().unwrap();
    assert_eq!(sent.friction_scale_factor, 0.8);
    assert_eq!(sent.wetness, "lowFlooded");
    assert_eq!(sent.properties.len(), 1);
    assert_eq!(sent.properties["swb.zone.ford.water_depth"], "0.12");
    // The server's refusal is SSv2's, word for word.
    let e = b
        .update_environment(&road(1.0, "dry", &[("swb.zone.nowhere.class", "ice")]))
        .unwrap_err();
    assert!(
        e.to_string().contains("zone 'nowhere' is not in the map"),
        "{e}"
    );
    // An environment without a road condition sends nothing.
    let n = seen.lock().unwrap().len();
    assert_eq!(b.update_environment(&Environment::default()), Ok(()));
    assert_eq!(seen.lock().unwrap().len(), n);
}

#[test]
fn the_weather_and_time_of_day_go_to_a_server_offering_them() {
    use crate::environment::{Precipitation, PrecipitationType, ScenarioWeather, TimeOfDay};
    let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let log = seen.clone();
    let mut inner = honest();
    let mut b = backend(&server(Box::new(move |req| {
        if let Some(wire::request::Request::SetEnvironment(e)) = &req.request {
            log.lock().unwrap().push(e.clone());
        }
        inner(req)
    })));
    b.initialize(&episode()).unwrap();
    let env = Environment {
        weather: Some(ScenarioWeather {
            cloud_cover_oktas: Some(4),
            precipitation: Some(Precipitation {
                kind: PrecipitationType::Rain,
                intensity: 5.0,
            }),
            ..Default::default()
        }),
        time_of_day: Some(TimeOfDay {
            hours: 18.5,
            day_of_year: 296,
            animation: true,
            date_time: "2026-10-23T18:30:00".into(),
        }),
        ..Default::default()
    };
    assert_eq!(b.update_environment(&env), Ok(()));
    let sent = seen.lock().unwrap().last().cloned().unwrap();
    let w = sent.weather.unwrap();
    assert_eq!(w.cloud_cover.unwrap().oktas, 4);
    assert!(w.sun.is_none() && w.fog.is_none() && w.wind.is_none());
    let p = w.precipitation.unwrap();
    assert_eq!(
        (p.r#type, p.intensity),
        (wire::scenario_weather::PrecipitationType::Rain as i32, 5.0)
    );
    let t = sent.time_of_day.unwrap();
    assert_eq!((t.hours, t.day_of_year, t.animation), (18.5, 296, true));
    // An empty Weather and no time: nothing to send.
    let n = seen.lock().unwrap().len();
    let empty = Environment {
        weather: Some(ScenarioWeather::default()),
        ..Default::default()
    };
    assert_eq!(b.update_environment(&empty), Ok(()));
    assert_eq!(seen.lock().unwrap().len(), n);
}

#[test]
fn nothing_is_sent_when_a_part_is_not_offered() {
    use crate::environment::TimeOfDay;
    const ROAD_ONLY: &[wire::Capability] = &[
        wire::Capability::TrackedMotion,
        wire::Capability::RoadCondition,
    ];
    let roads = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count = roads.clone();
    let mut inner = honest_with(ROAD_ONLY, "one");
    let mut b = backend(&server(Box::new(move |req| {
        if let Some(wire::request::Request::SetRoadCondition(_)) = &req.request {
            count.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        }
        inner(req)
    })));
    b.initialize(&episode()).unwrap();
    let mut env = road(0.5, "moist", &[]);
    env.time_of_day = Some(TimeOfDay {
        hours: 12.0,
        day_of_year: 1,
        animation: false,
        date_time: "2026-01-01T12:00:00".into(),
    });
    let e = b.update_environment(&env).unwrap_err();
    assert!(e.to_string().contains("does not offer ENVIRONMENT"), "{e}");
    // The road condition was not sent either.
    assert_eq!(roads.load(std::sync::atomic::Ordering::SeqCst), 0);
}

#[test]
fn a_road_condition_needs_the_capability_unless_it_changes_nothing() {
    const PLAIN: &[wire::Capability] = &[wire::Capability::TrackedMotion];
    let mut b = backend(&server(honest_with(PLAIN, "one")));
    b.initialize(&episode()).unwrap();
    assert_eq!(
        b.update_environment(&road(1.0, "dry", &[("other.tool", "x")])),
        Ok(())
    );
    let e = b.update_environment(&road(0.5, "dry", &[])).unwrap_err();
    assert!(
        e.to_string().contains("does not offer ROAD_CONDITION"),
        "{e}"
    );
}

#[test]
fn what_the_server_offers_is_asked_for() {
    let mut b = backend(&server(honest_with(CARLA_LIKE, "one")));
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("ap", Driver::SimulatorAutopilot, 0.0))
        .unwrap();
    assert_eq!(b.entities["ap"].mover, Mover::Simulator);
    let goal = Goal {
        waypoints: vec![pose(10.0, 0.0)],
        clear: false,
        target_speed: Some(5.0),
    };
    assert_eq!(b.update_entity_goal("ap", &goal), Ok(()));
    let bad = Goal {
        target_speed: Some(-1.0),
        ..goal
    };
    assert!(b.update_entity_goal("ap", &bad).is_err());
    let reply = b.get_weather();
    assert!(reply.success, "{}", reply.message);
    assert_eq!(
        reply.weather,
        crate::weather::preset("ClearNoon").map(|(_, w)| w)
    );
}

#[test]
fn a_server_that_does_not_answer_wedges_the_bridge() {
    let mut steps = 0;
    let mut inner = honest();
    let endpoint = server(Box::new(move |req| {
        if matches!(req.request, Some(wire::request::Request::Step(_))) {
            steps += 1;
            if steps == 2 {
                return None;
            }
        }
        inner(req)
    }));
    let mut b = backend(&endpoint);
    b.initialize(&episode()).unwrap();
    b.update_frame(0.05).unwrap();
    let e = b.update_frame(0.1).unwrap_err();
    assert!(
        e.to_string().contains("did not answer Step within 1 s"),
        "{e}"
    );
    assert!(b.wedged().is_some());
    // Nothing more goes to the server.
    assert!(b.update_frame(0.15).is_err());
}

#[test]
fn a_fatal_reply_wedges_the_bridge_at_once() {
    let mut inner = honest();
    let endpoint = server(Box::new(move |req| {
        if matches!(req.request, Some(wire::request::Request::Step(_))) {
            return Some(refused("the CARLA tick never returned", true));
        }
        inner(req)
    }));
    let mut b = backend(&endpoint);
    b.initialize(&episode()).unwrap();
    let e = b.update_frame(0.05).unwrap_err();
    assert!(e.to_string().contains("cannot go on"), "{e}");
    assert!(b.wedged().unwrap().contains("tick never returned"));
}

#[test]
fn a_refused_request_is_the_servers_reason() {
    let mut inner = honest();
    let endpoint = server(Box::new(move |req| {
        if matches!(req.request, Some(wire::request::Request::Spawn(_))) {
            return Some(refused("box dimensions 0, 0, 0 are not a box", false));
        }
        inner(req)
    }));
    let mut b = backend(&endpoint);
    b.initialize(&episode()).unwrap();
    let e = b.spawn(&vehicle("npc", Driver::Scenario, 0.0)).unwrap_err();
    assert_eq!(
        e,
        BackendError::Failed("Spawn 'npc': box dimensions 0, 0, 0 are not a box".into())
    );
    assert!(b.wedged().is_none());
}

#[test]
fn a_restarted_server_has_forgotten_every_entity_and_so_does_the_bridge() {
    let calls = std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0));
    let seen = calls.clone();
    let mut first = honest_with(ISAAC_LIKE, "one");
    let mut second = honest_with(ISAAC_LIKE, "two");
    let endpoint = server(Box::new(move |req| {
        // After the third request, a fresh server answers.
        if seen.fetch_add(1, std::sync::atomic::Ordering::SeqCst) < 3 {
            first(req)
        } else {
            second(req)
        }
    }));
    let mut b = backend(&endpoint);
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, 0.0)).unwrap();
    assert_eq!(calls.load(std::sync::atomic::Ordering::SeqCst), 3);
    b.probe_connection_idle();
    assert_eq!(b.entity_count(), 0);
    assert_eq!(b.server.instance, "two");
    assert!(b.server_lost().is_none());
}

#[test]
fn a_frame_is_the_fewest_physics_steps_the_server_takes() {
    let dt = 1.0 / 60.0;
    assert_eq!(physics_steps(0.05, dt).0, 3);
    assert!((physics_steps(0.05, dt).1 - dt).abs() < 1e-12);
    assert_eq!(physics_steps(0.1, dt).0, 6);
    assert_eq!(physics_steps(0.02, dt).0, 2);
    assert_eq!(physics_steps(dt, dt).0, 1);
    // A server that splits the step itself gets it whole.
    assert_eq!(physics_steps(0.05, 0.0), (1, 0.05));
}

#[test]
fn the_map_origin_comes_off_on_the_way_out_and_back_on_the_way_in() {
    let origin = [100.0, -50.0, 2.0];
    let p = pose(110.0, -45.0);
    let w = to_world(&p, origin);
    assert_eq!(w.position.unwrap().x, 10.0);
    assert_eq!(w.position.unwrap().y, 5.0);
    assert_eq!(w.position.unwrap().z, -2.0);
    assert_eq!(to_map(&w, origin), p);
    // A server's Euler angles are taken as they are.
    let e = wire::Pose {
        position: v3([1.0, 2.0, 3.0]),
        orientation: None,
        rpy: v3([0.1, 0.2, 0.3]),
    };
    assert_eq!(state_pose6(&e, origin), [101.0, -48.0, 5.0, 0.1, 0.2, 0.3]);
}

#[test]
fn a_world_file_names_the_world_beside_the_map() {
    let dir = std::env::temp_dir().join(format!("swb-world-file-{}", std::process::id()));
    std::fs::create_dir_all(dir.join("isaac")).unwrap();
    std::fs::create_dir_all(dir.join("carla")).unwrap();
    let map = dir.join("lanelet2_map.osm");
    let map = map.to_string_lossy();
    assert_eq!(
        WorldFile::for_map("isaac", &map).unwrap(),
        (WorldFile::default(), None)
    );
    // `usd` is the older name of `world`.
    std::fs::write(
        dir.join("isaac/world.yaml"),
        "usd: isaac/field.usd\nground_paths: [/World/env/terrain]\nmap_origin: [1.0, 2.0, 3.0]\n",
    )
    .unwrap();
    let (w, from) = WorldFile::for_map("isaac", &map).unwrap();
    assert_eq!(from, Some(dir.join("isaac/world.yaml")));
    assert_eq!(w.world, dir.join("isaac/field.usd").to_string_lossy());
    assert_eq!(w.ground_paths, vec!["/World/env/terrain".to_string()]);
    assert_eq!(w.map_origin, [1.0, 2.0, 3.0]);
    std::fs::write(dir.join("isaac/world.yaml"), "usd: x\nsky: blue\n").unwrap();
    assert!(WorldFile::for_map("isaac", &map).is_err());
    // CARLA: a town by name stays a name; the older map_origin.yaml still gives the origin.
    std::fs::write(dir.join("carla/map_origin.yaml"), "x: 5.0\ny: 6.0\n").unwrap();
    let (w, _) = WorldFile::for_map("carla", &map).unwrap();
    assert_eq!((w.world.as_str(), w.map_origin), ("", [5.0, 6.0, 0.0]));
    std::fs::write(dir.join("carla/world.yaml"), "world: Town10HD\n").unwrap();
    let (w, _) = WorldFile::for_map("carla", &map).unwrap();
    assert_eq!(w.world, "Town10HD");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn an_absent_box_is_the_kinds_default() {
    let (_, dims, declared) = entity_box(None, wire::Kind::Vehicle);
    assert_eq!((dims, declared), ([4.5, 2.1, 1.8], false));
    let flat = BoundingBox {
        center: None,
        dimensions: Some(crate::proto::geometry_msgs::Vector3 {
            x: 1.0,
            y: 0.0,
            z: 1.0,
        }),
    };
    let (c, d, declared) = entity_box(Some(&flat), wire::Kind::MiscObject);
    assert_eq!((c, d), default_box(wire::Kind::MiscObject));
    assert!(!declared);
}

fn moving(name: &str, x: f64, speed: f64) -> EntityStatus {
    use crate::proto::geometry_msgs::{Twist, Vector3};
    EntityStatus {
        name: name.into(),
        action_status: Some(ActionStatus {
            twist: Some(Twist {
                linear: Some(Vector3 {
                    x: speed,
                    y: 0.0,
                    z: 0.0,
                }),
                angular: Some(Vector3::default()),
            }),
            ..Default::default()
        }),
        pose: Some(pose(x, 0.0)),
    }
}

#[test]
fn a_tracked_npc_is_driven_by_velocity_and_follows_ssv2() {
    let mut b = backend_with(&server(honest()), NpcMotion::Tracked);
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, 0.0)).unwrap();
    assert!(b.entities["npc"].tracked.is_some());
    // SSv2 drives it along x at 8 m/s; the stand-in moves it at the velocity it is given.
    for frame in 1..=40u32 {
        let x = 8.0 * 0.05 * f64::from(frame);
        let out = b.update_entity_status(batch(vec![moving("npc", x, 8.0)]));
        assert_eq!(out.failure, None);
        // SSv2's status is echoed, not the body's.
        assert_eq!(out.entities[0].pose.unwrap().position.unwrap().x, x);
        let sent = &b.sent[&b.entities["npc"].handle];
        assert!(sent.twist.is_some() && !sent.kinematic);
        b.update_frame(0.05 * f64::from(frame)).unwrap();
        let body = b.entities["npc"].pose.position.unwrap().x;
        assert!(
            (body - x).abs() < 1e-6,
            "frame {frame}: body {body}, SSv2 {x}"
        );
    }
    let s = b.entities["npc"].tracked.as_ref().unwrap().stats.summary();
    assert_eq!((s.frames, s.snaps, s.fallback), (40, 0, 0));
    assert!(s.error_max < 1e-6, "{s:?}");
}

#[test]
fn a_tracked_npc_far_off_is_teleported() {
    let mut b = backend_with(&server(honest()), NpcMotion::Tracked);
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, 0.0)).unwrap();
    b.update_entity_status(batch(vec![moving("npc", 5.0, 0.0)]));
    let sent = &b.sent[&b.entities["npc"].handle];
    assert!(sent.teleport);
    assert_eq!(sent.pose.unwrap().position.unwrap().x, 5.0);
    b.update_frame(0.05).unwrap();
    assert_eq!(b.entities["npc"].pose.position.unwrap().x, 5.0);
}

#[test]
fn kinematic_npc_motion_keeps_posed_bodies() {
    let mut b = backend(&server(honest()));
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, 0.0)).unwrap();
    assert!(b.entities["npc"].tracked.is_none());
    b.update_entity_status(batch(vec![moving("npc", 0.4, 8.0)]));
    let sent = &b.sent[&b.entities["npc"].handle];
    assert!(sent.twist.is_none() && sent.pose.is_some());
}

#[test]
fn npc_contacts_are_their_own_and_hold_the_npc() {
    let mut b = backend_with(&server(honest()), NpcMotion::Tracked);
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, -3.0)).unwrap();
    let (ego, npc) = (b.entities["ego"].handle, b.entities["npc"].handle);
    b.frame = 7;
    b.record_contacts(&[
        wire::Contact {
            handle: ego,
            other: npc,
            impulse: 2.0,
            ..Default::default()
        },
        wire::Contact {
            handle: npc,
            other: ego,
            impulse: 2.0,
            ..Default::default()
        },
    ]);
    let run = b.ego_run.as_ref().unwrap();
    assert_eq!(run.contacts.len(), 1, "the ego's own contact, once");
    assert_eq!(run.npc_contacts.len(), 1);
    assert_eq!(run.npc_contacts[0].0, "npc");
    let t = b.entities["npc"].tracked.as_ref().unwrap();
    assert!(t.in_contact(7 + npc_tracking::CONTACT_HOLD_FRAMES));
    assert!(!t.in_contact(8 + npc_tracking::CONTACT_HOLD_FRAMES));
}

#[test]
fn a_contact_names_the_other_by_the_simulators_type_when_it_has_one() {
    let mut b = backend(&server(honest()));
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    let ego = b.entities["ego"].handle;
    b.record_contacts(&[wire::Contact {
        handle: ego,
        other: 0,
        other_sim_id: 77,
        other_type: "static.prop.streetbarrier".into(),
        impulse: 3.0,
        ..Default::default()
    }]);
    let c = &b.ego_run.as_ref().unwrap().contacts[0].2;
    assert_eq!(c.other_type, "static.prop.streetbarrier");
    assert_eq!(c.other_id, Some(77));
    assert!(c.is_static);
}

#[test]
fn a_one_frame_jolt_is_not_reported_as_acceleration() {
    let mut h = std::collections::VecDeque::new();
    for a in [0.1, 40.0, 0.2] {
        h.push_back([a, 0.0]);
    }
    assert_eq!(smoothed(&h), [0.2, 0.0]);
    h.pop_front();
    h.push_back([3.0, 0.0]);
    // Two frames agreeing pass: a sustained profile is not filtered away.
    h.pop_front();
    h.push_back([3.0, 0.0]);
    assert_eq!(smoothed(&h)[0], 3.0);
}

#[test]
fn signal_changes_go_to_the_world_server_by_opendrive_id() {
    use crate::proto::simulation_api_schema::{traffic_light, TrafficLight};
    let sent = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let log = sent.clone();
    let mut inner = honest();
    let endpoint = server(Box::new(move |req| {
        if let Some(wire::request::Request::SetSignals(s)) = &req.request {
            log.lock().unwrap().push(s.states.clone());
        }
        inner(req)
    }));
    let mut b = backend(&endpoint);
    b.initialize(&episode()).unwrap();
    b.signals = SignalTable::parse(
        "signals:\n- way_id: 43763\n  regulatory_element_ids: [43856]\n  opendrive_id: '377'\n",
    )
    .unwrap();
    let red = TrafficSignal {
        id: 43763,
        traffic_light_status: vec![TrafficLight {
            color: traffic_light::Color::Red as i32,
            shape: traffic_light::Shape::Circle as i32,
            status: traffic_light::Status::SolidOn as i32,
            confidence: 1.0,
        }],
        ..Default::default()
    };
    // Every frame carries it; only the change is sent.
    b.update_traffic_lights(std::slice::from_ref(&red)).unwrap();
    b.update_traffic_lights(std::slice::from_ref(&red)).unwrap();
    let sent = sent.lock().unwrap();
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0][0].opendrive_id, "377");
    assert_eq!(sent[0][0].colour, wire::Colour::Red as i32);
}

#[test]
fn a_misc_object_wears_its_model_or_the_street_barrier() {
    assert_eq!(
        misc_object_model("vehicle.mercedes.sprinter"),
        "vehicle.mercedes.sprinter"
    );
    assert_eq!(
        misc_object_model("static.prop.trafficcone01"),
        "static.prop.trafficcone01"
    );
    assert_eq!(misc_object_model(""), "static.prop.streetbarrier");
    assert_eq!(misc_object_model("obstacle"), "static.prop.streetbarrier");
}

#[test]
fn a_pedestrian_names_its_walker_only_when_the_key_is_one() {
    assert_eq!(
        pedestrian_model("walker.pedestrian.0001"),
        "walker.pedestrian.0001"
    );
    assert_eq!(pedestrian_model("pedestrian"), "");
    assert_eq!(pedestrian_model(""), "");
}
