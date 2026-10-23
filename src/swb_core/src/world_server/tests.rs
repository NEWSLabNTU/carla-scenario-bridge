//! The Rust shell on a fake world: the lease, handles, capabilities, the fatal end.

use super::*;

#[derive(Default)]
struct Fake {
    spawned: Vec<u32>,
    resets: Vec<Vec<u32>>,
    released: Vec<Vec<u32>>,
    fail_step_fatally: bool,
    caps: Vec<wire::Capability>,
}

impl World for Fake {
    fn simulator_name(&self) -> &str {
        "fake"
    }
    fn simulator_version(&self) -> String {
        "fake-0".into()
    }
    fn capabilities(&self) -> Vec<wire::Capability> {
        self.caps.clone()
    }
    fn load_world(&mut self, _r: &wire::LoadWorld) -> WorldResult<wire::LoadWorldReply> {
        self.spawned.clear();
        Ok(wire::LoadWorldReply { sim_time_ns: 7 })
    }
    fn spawn(&mut self, handle: u32, _r: &wire::Spawn) -> WorldResult<wire::Body> {
        self.spawned.push(handle);
        Ok(wire::Body::default())
    }
    fn despawn(&mut self, handle: u32) -> WorldResult<()> {
        self.spawned.retain(|h| *h != handle);
        Ok(())
    }
    fn apply(&mut self, _r: &wire::Apply) -> WorldResult<wire::ApplyReply> {
        Ok(wire::ApplyReply::default())
    }
    fn step(&mut self, _r: &wire::Step) -> WorldResult<wire::StepReply> {
        if self.fail_step_fatally {
            return Err(Refusal::fatal("the tick never returned"));
        }
        Ok(wire::StepReply {
            frame: 1,
            ..Default::default()
        })
    }
    fn reset(&mut self, handles: &[u32]) -> WorldResult<()> {
        self.resets.push(handles.to_vec());
        self.spawned.retain(|h| !handles.contains(h));
        Ok(())
    }
    fn release(&mut self, handles: &[u32]) -> WorldResult<()> {
        self.released.push(handles.to_vec());
        Ok(())
    }
}

fn id(pid: u32) -> wire::ClientId {
    wire::ClientId {
        host: "testhost".into(),
        pid,
        start_time: u64::from(pid),
    }
}

/// Clients whose pid is 999 are gone.
fn gone(c: &wire::ClientId) -> bool {
    c.pid == 999
}

fn call(shell: &mut Shell<Fake>, client: u32, request: wire::request::Request) -> wire::Reply {
    let bytes = wire::Request {
        request: Some(request),
        client: Some(id(client)),
    }
    .encode_to_vec();
    wire::Reply::decode(shell.handle(&bytes).as_slice()).unwrap()
}

fn hello(shell: &mut Shell<Fake>, client: u32, probe: bool) -> wire::Reply {
    call(
        shell,
        client,
        wire::request::Request::Hello(wire::Hello {
            protocol_version: PROTOCOL_VERSION,
            probe,
        }),
    )
}

fn spawn(shell: &mut Shell<Fake>, client: u32, name: &str, motion: wire::Motion) -> wire::Reply {
    call(
        shell,
        client,
        wire::request::Request::Spawn(wire::Spawn {
            name: name.into(),
            kind: wire::Kind::Vehicle as i32,
            motion: motion as i32,
            ..Default::default()
        }),
    )
}

fn shell(caps: Vec<wire::Capability>) -> Shell<Fake> {
    Shell::new(Fake {
        caps,
        ..Default::default()
    })
    .with_lease(Duration::from_millis(200), gone)
}

#[test]
fn hello_first_and_in_our_version() {
    let mut s = shell(vec![]);
    let r = call(
        &mut s,
        1,
        wire::request::Request::LoadWorld(Default::default()),
    );
    assert!(!r.ok);
    assert!(r.error.contains("before hello"), "{}", r.error);
    let r = call(
        &mut s,
        1,
        wire::request::Request::Hello(wire::Hello {
            protocol_version: PROTOCOL_VERSION + 1,
            probe: false,
        }),
    );
    assert!(!r.ok);
    let r = hello(&mut s, 1, false);
    assert!(r.ok, "{}", r.error);
    let Some(wire::reply::Reply::Hello(h)) = r.reply else {
        panic!("no hello reply");
    };
    assert_eq!(h.simulator_name, "fake");
    assert_eq!(h.owner.unwrap().pid, 1);
    assert!(h.ready);
}

#[test]
fn handles_are_the_shells_and_unique() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    let r = spawn(&mut s, 1, "a", wire::Motion::Kinematic);
    let Some(wire::reply::Reply::Spawn(a)) = r.reply else {
        panic!("{}", r.error);
    };
    assert_eq!(a.handle, 1);
    assert!(!spawn(&mut s, 1, "a", wire::Motion::Kinematic).ok);
    let r = call(
        &mut s,
        1,
        wire::request::Request::Despawn(wire::Despawn { handle: 9 }),
    );
    assert!(r.error.contains("no entity with handle 9"), "{}", r.error);
    let r = call(
        &mut s,
        1,
        wire::request::Request::Apply(wire::Apply {
            poses: vec![wire::EntityPose {
                handle: 9,
                ..Default::default()
            }],
            ..Default::default()
        }),
    );
    assert!(r.error.contains("unknown handle"), "{}", r.error);
    call(
        &mut s,
        1,
        wire::request::Request::LoadWorld(Default::default()),
    );
    let Some(wire::reply::Reply::Spawn(b)) = spawn(&mut s, 1, "a", wire::Motion::Kinematic).reply
    else {
        panic!("respawn after load");
    };
    assert_eq!(b.handle, 2);
}

#[test]
fn what_is_not_offered_is_refused() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    for (motion, needle) in [
        (wire::Motion::Tracked, "TRACKED bodies"),
        (wire::Motion::Autopilot, "AUTOPILOT bodies"),
        (wire::Motion::External, "other than the ego"),
    ] {
        let r = spawn(&mut s, 1, "x", motion);
        assert!(!r.ok && r.error.contains(needle), "{motion:?}: {}", r.error);
    }
    for q in [
        wire::request::Request::SetSignals(Default::default()),
        wire::request::Request::SetGoal(Default::default()),
        wire::request::Request::SetWeather(Default::default()),
        wire::request::Request::GetWeather(Default::default()),
        wire::request::Request::SetRoadCondition(Default::default()),
        wire::request::Request::SetEnvironment(Default::default()),
    ] {
        let r = call(&mut s, 1, q);
        assert!(!r.ok && r.error.contains("not offered"), "{}", r.error);
    }
    let mut s = shell(vec![
        wire::Capability::TracksTargets,
        wire::Capability::Autopilot,
    ]);
    hello(&mut s, 1, false);
    assert!(spawn(&mut s, 1, "t", wire::Motion::Tracked).ok);
    assert!(spawn(&mut s, 1, "a", wire::Motion::Autopilot).ok);
}

#[test]
fn one_owner_at_a_time() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    spawn(&mut s, 1, "npc", wire::Motion::Kinematic);
    // Another bridge: refused, and refused for every other request.
    let r = hello(&mut s, 2, false);
    assert!(
        !r.ok && r.error.contains("owned by testhost:1"),
        "{}",
        r.error
    );
    let r = call(&mut s, 2, wire::request::Request::Step(Default::default()));
    assert!(r.error.contains("owned by testhost:1"), "{}", r.error);
    // A probe is answered and changes nothing.
    let r = hello(&mut s, 2, true);
    assert!(r.ok);
    assert!(s.world.resets.is_empty());
    // The owner's own hello keeps its world.
    assert!(hello(&mut s, 1, false).ok);
    assert_eq!(s.world.spawned, vec![1]);
}

#[test]
fn a_dead_owner_is_replaced_at_once_and_a_silent_one_after_its_lease() {
    let mut s = shell(vec![]);
    hello(&mut s, 999, false);
    spawn(&mut s, 999, "npc", wire::Motion::Kinematic);
    let r = hello(&mut s, 2, false);
    assert!(r.ok, "{}", r.error);
    assert_eq!(s.world.resets, vec![vec![1]]);
    assert_eq!(s.owner().unwrap().pid, 2);
    // Pid 2 now owns it; pid 3 waits out the lease.
    assert!(!hello(&mut s, 3, false).ok);
    std::thread::sleep(Duration::from_millis(250));
    assert!(hello(&mut s, 3, false).ok);
    assert_eq!(s.owner().unwrap().pid, 3);
}

#[test]
fn release_leaves_no_owner() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    spawn(&mut s, 1, "npc", wire::Motion::Kinematic);
    assert!(
        call(
            &mut s,
            1,
            wire::request::Request::Release(Default::default())
        )
        .ok
    );
    assert_eq!(s.world.released, vec![vec![1]]);
    assert!(s.owner().is_none());
    assert!(hello(&mut s, 2, false).ok);
    assert!(s.world.resets.is_empty(), "nothing left to reset");
}

#[test]
fn a_fatal_call_ends_the_server() {
    let mut s = shell(vec![]);
    s.world.fail_step_fatally = true;
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, wire::request::Request::Step(Default::default()));
    assert!(!r.ok && r.fatal);
    assert_eq!(s.fatal(), Some("the tick never returned"));
    // Everything after is refused as fatal too.
    let r = call(&mut s, 1, wire::request::Request::Apply(Default::default()));
    assert!(r.fatal);
}
