//! Probe or check a world server over the world protocol (roadmap 046), whatever its
//! simulator and language.
//!
//!     csb_world_check hello <endpoint> [--wait SECONDS] [--quiet]
//!     csb_world_check conformance <endpoint> [--map <lanelet2_map.osm or map dir>]
//!     csb_world_check rtt <endpoint> [--n 2000] [--gap-us 0]
//!
//! `hello`: one probe `Hello` (changes nothing on the server: no ownership, no lease). Exit 0
//! with the server's description when it answers within 2 s, 1 when it does not (not
//! started, still starting, wedged); `--wait` retries that long. Safe beside a running
//! scenario.
//!
//! `conformance`: take the server over, as a bridge would, and run what every world server
//! must do -- the core (load, spawn, apply, step, despawn, the clock), the owner lease, and
//! each capability the server declares -- then release it. Exit 0 when every check passed.
//! It moves bodies in the server's world: run it between scenarios, never during one (a
//! bridge that owns the server makes it refuse, which is itself checked). `--map` is the map
//! to load (a CARLA town needs one); without it the server loads its default world.
//!
//! `rtt`: the round trip of a probe `Hello`, N times: the protocol's own cost per request on
//! this host (p50, p95, max), changing nothing on the server. `--gap-us`: a pause before
//! each, as a frame's idle time between requests (a sleeping server wakes slower).

use std::collections::BTreeSet;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use csb_core::proto::csb_world as wire;
use csb_core::world_client::{self, CallError, Client, PROTOCOL_VERSION};
use wire::request::Request as Q;

const TIMEOUT: Duration = Duration::from_secs(2);
const LOAD_TIMEOUT: Duration = Duration::from_secs(300);
const STEP: f64 = 0.05;

fn usage() -> ExitCode {
    eprintln!(
        "usage: csb_world_check hello <endpoint> [--wait SECONDS] [--quiet]\n       \
         csb_world_check conformance <endpoint> [--map <lanelet2 map>]\n       \
         csb_world_check rtt <endpoint> [--n N] [--gap-us US]"
    );
    ExitCode::from(2)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(cmd), Some(endpoint)) = (args.first(), args.get(1)) else {
        return usage();
    };
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    match cmd.as_str() {
        "hello" => hello(
            endpoint,
            opt("--wait").and_then(|w| w.parse().ok()).unwrap_or(0.0),
            args.iter().any(|a| a == "--quiet"),
        ),
        "conformance" => conformance(endpoint, opt("--map")),
        "rtt" => rtt(
            endpoint,
            opt("--n").and_then(|n| n.parse().ok()).unwrap_or(2000),
            Duration::from_micros(opt("--gap-us").and_then(|n| n.parse().ok()).unwrap_or(0)),
        ),
        _ => usage(),
    }
}

fn describe(h: &wire::HelloReply) -> String {
    let caps: Vec<&str> = h
        .capabilities
        .iter()
        .filter_map(|c| wire::Capability::try_from(*c).ok())
        .map(world_client::capability_name)
        .collect();
    format!(
        "{} ({}), protocol {}, capabilities {}, owner {}, {}",
        h.simulator_name,
        h.simulator_version,
        h.protocol_version,
        if caps.is_empty() {
            "core only".to_string()
        } else {
            caps.join(", ")
        },
        h.owner
            .as_ref()
            .map(world_client::describe)
            .unwrap_or_else(|| "none".into()),
        if h.ready {
            "ready".to_string()
        } else {
            format!("not ready: {}", h.not_ready_reason)
        }
    )
}

fn hello(endpoint: &str, wait: f64, quiet: bool) -> ExitCode {
    let deadline = Instant::now() + Duration::from_secs_f64(wait.max(0.0));
    let mut client = match Client::connect(endpoint) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{endpoint}: {e}");
            return ExitCode::from(1);
        }
    };
    loop {
        match client.hello(true, TIMEOUT) {
            Ok(h) => {
                if !quiet {
                    println!("world server at {endpoint}: up, {}", describe(&h));
                }
                return ExitCode::SUCCESS;
            }
            Err(e) => {
                if Instant::now() >= deadline {
                    if !quiet {
                        println!("world server at {endpoint}: {e}");
                    }
                    return ExitCode::from(1);
                }
                client.reset();
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

/// The checks' bookkeeping.
struct Report {
    passed: u32,
    failed: u32,
}

impl Report {
    fn check(&mut self, what: &str, result: Result<String, String>) {
        match result {
            Ok(note) => {
                self.passed += 1;
                println!(
                    "  PASS  {what}{}",
                    if note.is_empty() {
                        note
                    } else {
                        format!(": {note}")
                    }
                );
            }
            Err(why) => {
                self.failed += 1;
                println!("  FAIL  {what}: {why}");
            }
        }
    }

    fn skip(&mut self, what: &str, why: &str) {
        println!("  skip  {what}: {why}");
    }
}

fn v3(x: f64, y: f64, z: f64) -> Option<wire::Vec3> {
    Some(wire::Vec3 { x, y, z })
}

fn pose(x: f64, y: f64, z: f64) -> wire::Pose {
    wire::Pose {
        position: v3(x, y, z),
        orientation: Some(wire::Quat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }),
        rpy: None,
    }
}

fn call(c: &mut Client, q: Q, timeout: Duration) -> Result<wire::Reply, String> {
    c.call(q, timeout).map_err(|e| match e {
        CallError::Refused { error, .. } => format!("refused: {error}"),
        other => other.to_string(),
    })
}

fn spawn(
    c: &mut Client,
    name: &str,
    kind: wire::Kind,
    motion: wire::Motion,
    at: wire::Pose,
) -> Result<wire::SpawnReply, String> {
    let reply = call(
        c,
        Q::Spawn(wire::Spawn {
            name: name.into(),
            kind: kind as i32,
            subtype: if kind == wire::Kind::Pedestrian {
                "pedestrian".into()
            } else {
                "car".into()
            },
            box_center: v3(1.5, 0.0, 0.9),
            box_dimensions: v3(4.5, 2.1, 1.8),
            pose: Some(at),
            model: "vehicle.tesla.model3".into(),
            motion: motion as i32,
            asset_key: "vehicle.tesla.model3".into(),
            box_declared: true,
            ..Default::default()
        }),
        TIMEOUT * 10,
    )?;
    match reply.reply {
        Some(wire::reply::Reply::Spawn(s)) if s.handle != 0 => Ok(s),
        other => Err(format!("answered with {other:?}")),
    }
}

/// One SSv2 frame: Apply `poses`, then Step. The step's reply.
fn frame(
    c: &mut Client,
    poses: Vec<wire::EntityPose>,
    steps: (u32, f64),
) -> Result<(wire::ApplyReply, wire::StepReply), String> {
    let a = match call(
        c,
        Q::Apply(wire::Apply {
            poses,
            npc_logic_started: true,
            overwrite_ego_status: false,
        }),
        TIMEOUT * 5,
    )?
    .reply
    {
        Some(wire::reply::Reply::Apply(a)) => a,
        other => return Err(format!("Apply answered with {other:?}")),
    };
    let s = match call(
        c,
        Q::Step(wire::Step {
            substeps: steps.0,
            dt: steps.1,
            want_states: true,
            ..Default::default()
        }),
        TIMEOUT * 30,
    )?
    .reply
    {
        Some(wire::reply::Reply::Step(s)) => s,
        other => return Err(format!("Step answered with {other:?}")),
    };
    Ok((a, s))
}

fn state_x(s: &wire::StepReply, handle: u32) -> Option<f64> {
    s.states
        .iter()
        .find(|e| e.handle == handle)
        .and_then(|e| e.pose.as_ref())
        .and_then(|p| p.position)
        .map(|p| p.x)
}

fn conformance(endpoint: &str, map: Option<String>) -> ExitCode {
    let mut r = Report {
        passed: 0,
        failed: 0,
    };
    let mut c = match Client::connect(endpoint) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{endpoint}: {e}");
            return ExitCode::from(1);
        }
    };
    println!("World protocol {PROTOCOL_VERSION} conformance of {endpoint}");

    // --- Hello and the lease ------------------------------------------------------------
    let probe = match c.hello(true, TIMEOUT) {
        Ok(h) => h,
        Err(e) => {
            println!("  FAIL  hello: {e}");
            return ExitCode::from(1);
        }
    };
    println!("  {}", describe(&probe));
    if let Some(owner) = &probe.owner {
        println!(
            "  FAIL  the server is owned by {}: stop that bridge first (a conformance run moves \
             bodies in the world)",
            world_client::describe(owner)
        );
        return ExitCode::from(1);
    }
    let caps: BTreeSet<i32> = probe.capabilities.iter().copied().collect();
    let has = |c: wire::Capability| caps.contains(&(c as i32));
    r.check(
        "a wrong protocol version is refused",
        match c.call(
            Q::Hello(wire::Hello {
                protocol_version: PROTOCOL_VERSION + 1,
                probe: true,
            }),
            TIMEOUT,
        ) {
            Err(CallError::Refused { .. }) => Ok(String::new()),
            other => Err(format!("answered {other:?}")),
        },
    );
    r.check(
        "a request before hello is refused",
        match c.call(Q::Step(wire::Step::default()), TIMEOUT) {
            Err(CallError::Refused { error, .. }) => Ok(error),
            other => Err(format!("answered {other:?}")),
        },
    );
    let owned = c.hello(false, TIMEOUT);
    r.check(
        "hello takes ownership",
        match &owned {
            Ok(h) if h.owner.as_ref() == Some(c.id()) => Ok(String::new()),
            Ok(h) => Err(format!("owner is {:?}", h.owner)),
            Err(e) => Err(e.to_string()),
        },
    );
    if owned.is_err() {
        return ExitCode::from(1);
    }
    let instance = owned
        .as_ref()
        .map(|h| h.instance.clone())
        .unwrap_or_default();
    let mut other = Client::connect_as(
        endpoint,
        wire::ClientId {
            host: "csb-world-check-elsewhere".into(),
            pid: 4242,
            start_time: 1,
        },
    )
    .expect("connect");
    r.check(
        "a second bridge is refused while the owner lives",
        match other.hello(false, TIMEOUT) {
            Err(CallError::Refused { error, .. }) if error.contains("owned by") => Ok(error),
            other => Err(format!("answered {other:?}")),
        },
    );
    r.check(
        "a second bridge's requests are refused",
        match other.call(Q::Step(wire::Step::default()), TIMEOUT) {
            Err(CallError::Refused { .. }) => Ok(String::new()),
            other => Err(format!("answered {other:?}")),
        },
    );
    r.check(
        "a probe changes nothing",
        match other.hello(true, TIMEOUT) {
            Ok(h) if h.owner.as_ref() == Some(c.id()) && h.instance == instance => {
                Ok(String::new())
            }
            other => Err(format!("answered {other:?}")),
        },
    );

    // --- The core -----------------------------------------------------------------------
    if !owned.as_ref().is_ok_and(|h| h.ready) {
        let mut ready = false;
        for _ in 0..30 {
            if c.hello(false, TIMEOUT).is_ok_and(|h| h.ready) {
                ready = true;
                break;
            }
            std::thread::sleep(Duration::from_secs(2));
        }
        if !ready {
            r.check(
                "the server becomes ready",
                Err("not ready after 60 s".into()),
            );
            return finish(c, r);
        }
    }
    let max_dt = owned.as_ref().map(|h| h.max_physics_dt).unwrap_or(0.0);
    let steps = if max_dt > 0.0 {
        let n = ((STEP / max_dt) - 1e-9).ceil().max(1.0) as u32;
        (n, STEP / f64::from(n))
    } else {
        (1, STEP)
    };
    let (map_path, map_dir) = match &map {
        Some(m) if m.ends_with(".osm") => (
            m.clone(),
            std::path::Path::new(m)
                .parent()
                .map(|p| p.to_string_lossy().into_owned())
                .unwrap_or_default(),
        ),
        Some(m) => (format!("{m}/lanelet2_map.osm"), m.clone()),
        None => (String::new(), String::new()),
    };
    let load = || {
        Q::LoadWorld(wire::LoadWorld {
            lanelet2_map_path: map_path.clone(),
            map_dir: map_dir.clone(),
            map_origin: v3(0.0, 0.0, 0.0),
            step_time: STEP,
            realtime_factor: 1.0,
            ..Default::default()
        })
    };
    let t0 = match call(&mut c, load(), LOAD_TIMEOUT) {
        Ok(wire::Reply {
            reply: Some(wire::reply::Reply::LoadWorld(l)),
            ..
        }) => {
            r.check(
                "load a world",
                Ok(format!("time {:.3} s", l.sim_time_ns as f64 / 1e9)),
            );
            l.sim_time_ns
        }
        Ok(other) => {
            r.check("load a world", Err(format!("answered {other:?}")));
            return finish(c, r);
        }
        Err(e) => {
            r.check("load a world", Err(e));
            return finish(c, r);
        }
    };
    // Where to put bodies: the map origin's surroundings are not drivable everywhere, so
    // only relative motion is checked.
    let (x0, y0, z0) = (0.0, 0.0, 0.5);
    let ego = match spawn(
        &mut c,
        "conformance_ego",
        wire::Kind::Ego,
        wire::Motion::External,
        pose(x0, y0, z0),
    ) {
        Ok(s) => {
            r.check(
                "spawn the ego",
                Ok(format!(
                    "handle {}, {}",
                    s.handle,
                    s.body.map(|b| b.model).unwrap_or_default()
                )),
            );
            Some(s.handle)
        }
        Err(e) => {
            r.check("spawn the ego", Err(e));
            None
        }
    };
    let npc = spawn(
        &mut c,
        "conformance_npc",
        wire::Kind::Vehicle,
        wire::Motion::Kinematic,
        pose(x0 + 20.0, y0, z0),
    );
    r.check(
        "spawn a kinematic vehicle",
        npc.as_ref()
            .map(|s| format!("handle {}", s.handle))
            .map_err(Clone::clone),
    );
    r.check(
        "a second entity of one name is refused",
        match spawn(
            &mut c,
            "conformance_npc",
            wire::Kind::Vehicle,
            wire::Motion::Kinematic,
            pose(x0 + 40.0, y0, z0),
        ) {
            Err(e) if e.contains("refused") => Ok(e),
            other => Err(format!("answered {other:?}")),
        },
    );
    let mut last_ns = t0;
    if let Ok(npc) = &npc {
        let h = npc.handle;
        let mut moved = Vec::new();
        let mut times_ok = true;
        let mut failure = None;
        for k in 1..=5 {
            let mut poses = vec![wire::EntityPose {
                handle: h,
                pose: Some(pose(x0 + 20.0 + f64::from(k), y0, z0)),
                ..Default::default()
            }];
            if let Some(e) = ego {
                poses.push(wire::EntityPose {
                    handle: e,
                    ..Default::default()
                });
            }
            match frame(&mut c, poses, steps) {
                Ok((_, s)) => {
                    let ns = if s.sim_time_ns != 0 {
                        s.sim_time_ns
                    } else {
                        (s.sim_time * 1e9).round() as i64
                    };
                    times_ok &= ns > last_ns;
                    last_ns = ns;
                    moved.push(state_x(&s, h));
                }
                Err(e) => {
                    failure = Some(e);
                    break;
                }
            }
        }
        r.check(
            "five frames of Apply and Step",
            failure.map_or(Ok(String::new()), Err),
        );
        r.check(
            "time advances every step",
            if times_ok {
                Ok(format!("{:.3} s", last_ns as f64 / 1e9))
            } else {
                Err("it did not".into())
            },
        );
        let xs: Vec<f64> = moved.iter().flatten().copied().collect();
        r.check(
            "a kinematic body goes where it is posed",
            match xs.as_slice() {
                [first, .., last] if (last - first - 4.0).abs() < 0.5 => {
                    Ok(format!("{:.2} m in 4 frames", last - first))
                }
                [] => Err("no state for it in the step replies".into()),
                other => Err(format!("its x went {other:?}")),
            },
        );
    }
    r.check(
        "despawning an unknown handle is refused",
        match call(
            &mut c,
            Q::Despawn(wire::Despawn { handle: 999_999 }),
            TIMEOUT,
        ) {
            Err(e) if e.contains("refused") => Ok(String::new()),
            other => Err(format!("answered {other:?}")),
        },
    );

    // --- Capabilities --------------------------------------------------------------------
    let tracked_kind = if has(wire::Capability::TracksTargets) {
        Some("TRACKS_TARGETS")
    } else if has(wire::Capability::TrackedMotion) {
        Some("TRACKED_MOTION")
    } else {
        None
    };
    match tracked_kind {
        None => r.skip(
            "tracked motion",
            "neither TRACKED_MOTION nor TRACKS_TARGETS",
        ),
        Some(how) => {
            match spawn(
                &mut c,
                "conformance_tracked",
                wire::Kind::Vehicle,
                wire::Motion::Tracked,
                pose(x0 + 60.0, y0, z0),
            ) {
                Err(e) => r.check(&format!("spawn a tracked body ({how})"), Err(e)),
                Ok(s) => {
                    let mut xs = Vec::new();
                    let mut failure = None;
                    for k in 1..=20 {
                        let target = x0 + 60.0 + 5.0 * STEP * f64::from(k);
                        let p = if how == "TRACKS_TARGETS" {
                            wire::EntityPose {
                                handle: s.handle,
                                pose: Some(pose(target, y0, z0)),
                                ssv2_twist: Some(wire::Twist {
                                    linear: v3(5.0, 0.0, 0.0),
                                    angular: v3(0.0, 0.0, 0.0),
                                }),
                                ..Default::default()
                            }
                        } else {
                            wire::EntityPose {
                                handle: s.handle,
                                twist: Some(wire::Twist {
                                    linear: v3(5.0, 0.0, 0.0),
                                    angular: v3(0.0, 0.0, 0.0),
                                }),
                                ..Default::default()
                            }
                        };
                        match frame(&mut c, vec![p], steps) {
                            Ok((a, st)) => {
                                if how == "TRACKS_TARGETS"
                                    && k == 20
                                    && !a.tracking.iter().any(|t| t.handle == s.handle)
                                {
                                    failure = Some("no TrackingSample for it".to_string());
                                }
                                xs.push(state_x(&st, s.handle));
                            }
                            Err(e) => {
                                failure = Some(e);
                                break;
                            }
                        }
                    }
                    let xs: Vec<f64> = xs.into_iter().flatten().collect();
                    r.check(
                        &format!("a tracked body follows at 5 m/s ({how})"),
                        match (failure, xs.first(), xs.last()) {
                            (Some(e), _, _) => Err(e),
                            (None, Some(a), Some(b)) if (b - a) > 2.5 => {
                                Ok(format!("{:.2} m in {} frames", b - a, xs.len() - 1))
                            }
                            (None, a, b) => Err(format!("it went from {a:?} to {b:?}")),
                        },
                    );
                    let _ = call(
                        &mut c,
                        Q::Despawn(wire::Despawn { handle: s.handle }),
                        TIMEOUT * 5,
                    );
                }
            }
        }
    }
    if has(wire::Capability::Signals) {
        r.check(
            "signal colours are taken (SIGNALS)",
            call(
                &mut c,
                Q::SetSignals(wire::SetSignals { states: vec![] }),
                TIMEOUT,
            )
            .map(|_| String::new()),
        );
    } else {
        r.skip("signals", "no SIGNALS");
        r.check(
            "signals are refused without SIGNALS",
            match call(&mut c, Q::SetSignals(wire::SetSignals::default()), TIMEOUT) {
                Err(e) if e.contains("refused") => Ok(String::new()),
                other => Err(format!("answered {other:?}")),
            },
        );
    }
    if has(wire::Capability::Weather) {
        let set = call(
            &mut c,
            Q::SetWeather(wire::SetWeather {
                weather: Some(wire::Weather {
                    preset: "ClearNoon".into(),
                    ..Default::default()
                }),
            }),
            LOAD_TIMEOUT,
        );
        let got = call(&mut c, Q::GetWeather(wire::GetWeather {}), TIMEOUT * 5);
        r.check(
            "the weather is set and read back (WEATHER)",
            match (set, got) {
                (
                    Ok(_),
                    Ok(wire::Reply {
                        reply: Some(wire::reply::Reply::Weather(w)),
                        ..
                    }),
                ) if w.weather.as_ref().is_some_and(|x| x.preset == "ClearNoon") => {
                    Ok(String::new())
                }
                (a, b) => Err(format!("set {a:?}, get {b:?}")),
            },
        );
    } else {
        r.skip("weather", "no WEATHER");
    }
    if has(wire::Capability::Autopilot) {
        r.skip("autopilot", "needs a road; the scenario suite checks it");
    }

    // --- The world again, and the end ---------------------------------------------------------
    match call(&mut c, load(), LOAD_TIMEOUT) {
        Ok(wire::Reply {
            reply: Some(wire::reply::Reply::LoadWorld(l)),
            ..
        }) => r.check(
            "time goes on across a load",
            if l.sim_time_ns >= last_ns {
                Ok(String::new())
            } else {
                Err(format!("{} after {last_ns}", l.sim_time_ns))
            },
        ),
        other => r.check("load the world again", Err(format!("{other:?}"))),
    }
    finish(c, r)
}

fn finish(mut c: Client, mut r: Report) -> ExitCode {
    let released = call(
        &mut c,
        Q::Release(wire::Release {}),
        Duration::from_secs(60),
    );
    r.check("release", released.map(|_| String::new()));
    r.check(
        "no owner after release",
        match c.hello(true, TIMEOUT) {
            Ok(h) if h.owner.is_none() => Ok(String::new()),
            other => Err(format!("{other:?}")),
        },
    );
    println!("{} passed, {} failed", r.passed, r.failed);
    if r.failed == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

fn rtt(endpoint: &str, n: usize, gap: Duration) -> ExitCode {
    let mut client = match Client::connect(endpoint) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{endpoint}: {e}");
            return ExitCode::from(1);
        }
    };
    let mut times = Vec::with_capacity(n);
    for _ in 0..n.max(1) {
        if !gap.is_zero() {
            std::thread::sleep(gap);
        }
        let started = Instant::now();
        if let Err(e) = client.hello(true, TIMEOUT) {
            eprintln!("{endpoint}: {e}");
            return ExitCode::from(1);
        }
        times.push(started.elapsed().as_secs_f64() * 1e6);
    }
    times.sort_by(f64::total_cmp);
    let at = |q: f64| times[((q * (times.len() - 1) as f64).round() as usize).min(times.len() - 1)];
    println!(
        "{} probe hellos to {endpoint}: round trip p50 {:.0} us, p95 {:.0} us, max {:.0} us",
        times.len(),
        at(0.5),
        at(0.95),
        at(1.0)
    );
    ExitCode::SUCCESS
}
