//! `IsaacBackend` against an in-process world server: a REP socket on a thread that keeps
//! poses exactly, steps a clock, and reports a contact while the ego and another body are
//! within 1 m.

use super::*;
use crate::proto::geometry_msgs::{Point, Quaternion};
use std::collections::HashMap;

/// What the stand-in server does with a request; `None` never answers.
type Behaviour = Box<dyn FnMut(&wire::Request) -> Option<wire::Reply> + Send>;

fn ok(reply: Option<wire::reply::Reply>) -> wire::Reply {
    wire::Reply {
        ok: true,
        error: String::new(),
        reply,
    }
}

/// A server that behaves as the real one does, as far as the backend can tell.
fn honest() -> Behaviour {
    let mut next = 1u32;
    let mut bodies: HashMap<u32, (wire::Pose, bool)> = HashMap::new();
    let mut time = 0.0;
    let mut frame = 0u64;
    Box::new(move |req| {
        use wire::request::Request as R;
        Some(match req.request.clone()? {
            R::Hello(h) => ok(Some(wire::reply::Reply::Hello(wire::HelloReply {
                protocol_version: h.protocol_version,
                isaac_version: "stand-in".into(),
            }))),
            R::LoadWorld(_) => {
                bodies.clear();
                ok(None)
            }
            R::Spawn(s) => {
                let h = next;
                next += 1;
                bodies.insert(h, (s.pose.unwrap(), s.kind == wire::Kind::Ego as i32));
                ok(Some(wire::reply::Reply::Spawn(wire::SpawnReply {
                    handle: h,
                })))
            }
            R::Despawn(d) => {
                bodies.remove(&d.handle);
                ok(None)
            }
            R::Step(s) => {
                for p in s.poses {
                    bodies.get_mut(&p.handle).unwrap().0 = p.pose.unwrap();
                }
                time += s.substeps as f64 * s.dt;
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
                        })
                        .collect(),
                    contacts,
                })))
            }
            R::Shutdown(_) => ok(None),
        })
    })
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

fn backend(endpoint: &str) -> IsaacBackend {
    let mut config = BridgeConfig::default();
    config.isaac.endpoint = endpoint.to_string();
    config.isaac.request_timeout_seconds = 1;
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

#[test]
fn a_frame_carries_the_poses_and_reads_the_ego_back() {
    let mut b = backend(&server(honest()));
    assert_eq!(b.initialize(&episode()), Ok(0));
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, -10.0)).unwrap();
    assert_eq!(b.entity_count(), 2);

    // The NPC's pose goes out with the next step; the ego, not driven by SSv2, is read back.
    for frame in 1..=3 {
        let out = b.update_entity_status(StatusBatch {
            entities: vec![status("ego", 99.0), status("npc", -10.0 + frame as f64)],
            npc_logic_started: true,
            overwrite_ego_status: false,
        });
        assert_eq!(out.failure, None);
        let ego = out.entities.iter().find(|e| e.name == "ego").unwrap();
        assert_eq!(ego.pose.unwrap().position.unwrap().x, 0.0, "the ego stands");
        let t = b.update_frame(0.05 * frame as f64).unwrap();
        assert_eq!(t, 50_000_000 * frame);
    }
    assert_eq!(b.entities["npc"].pose.position.unwrap().x, -7.0);
    // 1 m/s along x, 20 frames a second: the NPC's motion is differenced from its poses.
    assert!((b.entities["npc"].motion.body[0] - 20.0).abs() < 1e-9);
    assert!(b.take_tick_time() > Duration::ZERO);

    // SSv2 drives the ego (a trajectory): its pose goes out too, and comes back from Isaac.
    b.update_entity_status(StatusBatch {
        entities: vec![status("ego", 2.0)],
        npc_logic_started: true,
        overwrite_ego_status: true,
    });
    b.update_frame(0.2).unwrap();
    assert_eq!(b.entities["ego"].pose.position.unwrap().x, 2.0);
}

#[test]
fn the_egos_contacts_are_counted_per_other_entity() {
    let mut b = backend(&server(honest()));
    b.initialize(&episode()).unwrap();
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    b.spawn(&vehicle("npc", Driver::Scenario, -3.0)).unwrap();
    for x in [-2.0, -0.5, 0.0, 0.5, 2.0] {
        b.update_entity_status(StatusBatch {
            entities: vec![status("npc", x)],
            npc_logic_started: true,
            overwrite_ego_status: false,
        });
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
fn what_isaac_does_not_have_is_an_honest_failure() {
    let mut b = backend(&server(honest()));
    b.initialize(&episode()).unwrap();
    for driver in [Driver::SimulatorAutopilot, Driver::Agent] {
        let e = b.spawn(&vehicle("bg", driver, 0.0)).unwrap_err();
        assert!(matches!(e, BackendError::Unsupported(_)), "{e:?}");
    }
    assert_eq!(b.update_traffic_lights(&[]), Ok(()));
    let light = TrafficSignal::default();
    assert!(matches!(
        b.update_traffic_lights(&[light]),
        Err(BackendError::Unsupported(_))
    ));
    assert!(matches!(
        b.attach_sensor(Sensor::Lidar),
        Err(BackendError::Unsupported(_))
    ));
    assert!(!b.get_weather().success);
    // Unknown names fail, by name.
    let out = b.update_entity_status(StatusBatch {
        entities: vec![status("ghost", 0.0)],
        npc_logic_started: true,
        overwrite_ego_status: false,
    });
    assert!(out.failure.unwrap().contains("'ghost'"));
    assert!(b
        .despawn("ghost")
        .unwrap_err()
        .to_string()
        .contains("'ghost' not found"));
    // One ego.
    b.spawn(&vehicle("ego", Driver::Ego, 0.0)).unwrap();
    let e = b.spawn(&vehicle("ego2", Driver::Ego, 5.0)).unwrap_err();
    assert!(e.to_string().contains("ego 'ego' already exists"), "{e}");
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
fn a_refused_request_is_the_servers_reason() {
    let mut inner = honest();
    let endpoint = server(Box::new(move |req| {
        if matches!(req.request, Some(wire::request::Request::Spawn(_))) {
            return Some(wire::Reply {
                ok: false,
                error: "box dimensions 0, 0, 0 are not a box".into(),
                reply: None,
            });
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
fn a_frame_is_the_fewest_physics_steps_of_at_most_60_hz() {
    assert_eq!(physics_steps(0.05).0, 3);
    assert!((physics_steps(0.05).1 - 1.0 / 60.0).abs() < 1e-12);
    assert_eq!(physics_steps(0.1).0, 6);
    assert_eq!(physics_steps(0.02).0, 2);
    assert_eq!(physics_steps(1.0 / 60.0).0, 1);
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
}

#[test]
fn a_world_file_names_the_usd_beside_the_map() {
    let dir = std::env::temp_dir().join(format!("csb-isaac-world-{}", std::process::id()));
    std::fs::create_dir_all(dir.join("isaac")).unwrap();
    let map = dir.join("lanelet2_map.osm");
    assert_eq!(
        WorldFile::for_map(&map.to_string_lossy()).unwrap(),
        (WorldFile::default(), None)
    );
    std::fs::write(
        dir.join(WORLD_SUBPATH),
        "usd: isaac/field.usd\nground_paths: [/World/env/terrain]\nmap_origin: [1.0, 2.0, 3.0]\n",
    )
    .unwrap();
    let (w, from) = WorldFile::for_map(&map.to_string_lossy()).unwrap();
    assert_eq!(from, Some(dir.join(WORLD_SUBPATH)));
    assert_eq!(w.usd, dir.join("isaac/field.usd").to_string_lossy());
    assert_eq!(w.ground_paths, vec!["/World/env/terrain".to_string()]);
    assert_eq!(w.map_origin, [1.0, 2.0, 3.0]);
    std::fs::write(dir.join(WORLD_SUBPATH), "usd: x\nsky: blue\n").unwrap();
    assert!(WorldFile::for_map(&map.to_string_lossy()).is_err());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn an_absent_box_is_the_kinds_default() {
    assert_eq!(entity_box(None, wire::Kind::Vehicle).1, [4.5, 2.1, 1.8]);
    let flat = BoundingBox {
        center: None,
        dimensions: Some(crate::proto::geometry_msgs::Vector3 {
            x: 1.0,
            y: 0.0,
            z: 1.0,
        }),
    };
    assert_eq!(
        entity_box(Some(&flat), wire::Kind::MiscObject),
        default_box(wire::Kind::MiscObject)
    );
}
