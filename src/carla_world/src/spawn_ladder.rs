//! One entity's spawn: the lift ladder for a collision, adoption for a timeout (roadmap 034
//! step 4).
//!
//! CARLA finishes a spawn whatever its client does: the RPC is queued to the game thread and
//! runs there even after the client gave up (`CarlaServer.cpp:725-746`; in asynchronous mode
//! the game thread serves RPCs for 1 ms a frame, which is how one ego spawn took 31 s on a
//! loaded host, 033 step 4b). LibCarla's `World::TrySpawnActor` swallows every exception, so
//! a timeout read exactly like a collision, and the ladder lifted a second `hero` onto the
//! first. Spawning through `apply_batch_sync` tells them apart: a collision is that
//! command's error ("Spawn failed because of collision at spawn position",
//! `ActorSpawnResult.cpp:22-23`); a timeout fails the whole call (`Client.cpp:753-758`).
//!
//! After a failed call the actor may exist. Actor ids only increase (`ActorRegistry.cpp:24`,
//! never reset), so csb notes the highest id it knows before the spawn and looks for an
//! actor of the right blueprint and role above it; the lookup is an RPC queued behind the
//! spawn on the same game thread, so it sees the spawn if the server ran it. Found: adopted,
//! never lifted over. Not found: an error, never a lift.
//!
//! The decisions are here, testable without CARLA; the coordinator supplies the calls.

use std::time::{Duration, Instant};

use crate::budgets::Deadline;

/// What one spawn call returned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Attempt {
    /// CARLA created the actor.
    Spawned(u32),
    /// CARLA refused it: something occupies the point.
    Collision(String),
    /// CARLA refused it for another reason (a bad description): lifting cannot help.
    Refused(String),
    /// The call itself failed -- a timeout, a lost connection: the actor may exist.
    CallFailed(String),
}

/// The CARLA side of a spawn.
pub trait SpawnBackend {
    /// One spawn at height `z`, its RPC bounded by `timeout`.
    fn spawn(&mut self, z: f32, timeout: Duration) -> Attempt;
    /// Ids above `watermark` of actors with this spawn's blueprint and role, ascending.
    fn find_late(&mut self, watermark: u32, timeout: Duration) -> Result<Vec<u32>, String>;
}

/// How a spawn ended.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Spawned on attempt `attempt` (0-based) at `z`.
    Spawned { actor: u32, attempt: usize, z: f32 },
    /// The call of attempt `attempt` failed, and the actor it created was found and adopted.
    Adopted {
        actor: u32,
        attempt: usize,
        z: f32,
        error: String,
        others: Vec<u32>,
    },
    /// No actor; why.
    Failed(String),
    /// No actor: every attempt of the ladder collided. Something occupies the point -- a
    /// vehicle, or the static world (the caller can tell which); the last error.
    Blocked(String),
}

/// The lookup after a failed call may take this long.
pub const ADOPT_LOOKUP_TIMEOUT: Duration = Duration::from_secs(10);
/// Kept free after an attempt for what follows it in the request: the settling tick (its
/// floor) and the state read after it, should it hang.
pub const AFTER_SPAWN_RESERVE: Duration = Duration::from_secs(20);
/// An attempt is not started with less than this for its RPC.
pub const MIN_ATTEMPT: Duration = Duration::from_secs(2);

/// Run the ladder over `heights` under `deadline`. `rpc_timeout` is the client's usual RPC
/// timeout; each attempt gets that, or what is left once the lookup and the rest of the
/// request are reserved. `now` reads the clock (a parameter for the tests).
pub fn run(
    backend: &mut impl SpawnBackend,
    heights: impl IntoIterator<Item = f32>,
    watermark: u32,
    deadline: &Deadline,
    rpc_timeout: Duration,
    mut now: impl FnMut() -> Instant,
) -> Outcome {
    let mut last_collision = None;
    let mut attempts = 0;
    for (attempt, z) in heights.into_iter().enumerate() {
        let left = deadline
            .remaining_at(now())
            .saturating_sub(ADOPT_LOOKUP_TIMEOUT + AFTER_SPAWN_RESERVE);
        let timeout = rpc_timeout.min(left);
        if timeout < MIN_ATTEMPT {
            return Outcome::Failed(format!(
                "out of time after {attempts} attempt(s) at increasing height (the request's \
                 {} s budget){}",
                deadline.budget().as_secs(),
                last_collision
                    .map(|e| format!("; last error: {e}"))
                    .unwrap_or_default()
            ));
        }
        attempts += 1;
        match backend.spawn(z, timeout) {
            Attempt::Spawned(actor) => return Outcome::Spawned { actor, attempt, z },
            Attempt::Collision(e) => last_collision = Some(e),
            Attempt::Refused(e) => return Outcome::Failed(format!("CARLA refused it: {e}")),
            Attempt::CallFailed(error) => {
                let lookup = ADOPT_LOOKUP_TIMEOUT.min(deadline.remaining_at(now()));
                return match backend.find_late(watermark, lookup.max(MIN_ATTEMPT)) {
                    Ok(found) if !found.is_empty() => Outcome::Adopted {
                        actor: found[0],
                        attempt,
                        z,
                        error,
                        others: found[1..].to_vec(),
                    },
                    Ok(_) => Outcome::Failed(format!(
                        "the spawn call failed ({error}) and no actor of this blueprint and \
                         role appeared above id {watermark}: CARLA did not create it"
                    )),
                    Err(e) => Outcome::Failed(format!(
                        "the spawn call failed ({error}), and looking for the actor it may \
                         have created failed too ({e})"
                    )),
                };
            }
        }
    }
    Outcome::Blocked(format!(
        "{attempts} attempt(s) at increasing height collided; last error: {}",
        last_collision.unwrap_or_else(|| "none".into())
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fake CARLA: answers spawns from a script, holds the actors a timed-out spawn
    /// created anyway.
    #[derive(Default)]
    struct Fake {
        script: Vec<Attempt>,
        late: Vec<u32>,
        lookups: Vec<u32>,
        spawned_at: Vec<f32>,
        timeouts: Vec<Duration>,
    }

    impl SpawnBackend for Fake {
        fn spawn(&mut self, z: f32, timeout: Duration) -> Attempt {
            self.spawned_at.push(z);
            self.timeouts.push(timeout);
            self.script.remove(0)
        }
        fn find_late(&mut self, watermark: u32, _: Duration) -> Result<Vec<u32>, String> {
            self.lookups.push(watermark);
            Ok(self
                .late
                .iter()
                .copied()
                .filter(|&id| id > watermark)
                .collect())
        }
    }

    const HEIGHTS: [f32; 5] = [0.3, 0.8, 1.3, 1.8, 2.3];
    const RPC: Duration = Duration::from_secs(30);

    fn go(fake: &mut Fake, deadline: &Deadline) -> Outcome {
        run(fake, HEIGHTS, 9000, deadline, RPC, Instant::now)
    }

    #[test]
    fn a_timeout_adopts_the_late_actor_and_never_lifts() {
        // 033 step 4b: the first attempt timed out after CARLA had created 9004.
        let mut fake = Fake {
            script: vec![Attempt::CallFailed("rpc timeout".into())],
            late: vec![8990, 9004],
            ..Default::default()
        };
        let out = go(&mut fake, &Deadline::new(Duration::from_secs(75)));
        assert_eq!(
            out,
            Outcome::Adopted {
                actor: 9004,
                attempt: 0,
                z: 0.3,
                error: "rpc timeout".into(),
                others: vec![],
            }
        );
        assert_eq!(fake.spawned_at, vec![0.3], "no second spawn, no lift");
        assert_eq!(fake.lookups, vec![9000]);
    }

    #[test]
    fn a_timeout_with_no_late_actor_is_an_error_not_a_lift() {
        let mut fake = Fake {
            script: vec![Attempt::CallFailed("rpc timeout".into())],
            ..Default::default()
        };
        let Outcome::Failed(why) = go(&mut fake, &Deadline::new(Duration::from_secs(75))) else {
            panic!("expected a failure");
        };
        assert!(why.contains("did not create it"), "{why}");
        assert_eq!(fake.spawned_at.len(), 1);
    }

    #[test]
    fn a_collision_climbs_the_ladder_as_before() {
        let collision = || Attempt::Collision("Spawn failed because of collision".into());
        let mut fake = Fake {
            script: vec![collision(), collision(), Attempt::Spawned(9001)],
            ..Default::default()
        };
        let out = go(&mut fake, &Deadline::new(Duration::from_secs(75)));
        assert_eq!(
            out,
            Outcome::Spawned {
                actor: 9001,
                attempt: 2,
                z: 1.3
            }
        );
        assert_eq!(fake.spawned_at, vec![0.3, 0.8, 1.3]);
        assert!(fake.lookups.is_empty());
    }

    #[test]
    fn a_collision_then_a_timeout_adopts_at_the_lifted_height() {
        let mut fake = Fake {
            script: vec![
                Attempt::Collision("collision".into()),
                Attempt::CallFailed("rpc timeout".into()),
            ],
            late: vec![9002],
            ..Default::default()
        };
        let out = go(&mut fake, &Deadline::new(Duration::from_secs(75)));
        assert!(matches!(
            out,
            Outcome::Adopted {
                actor: 9002,
                attempt: 1,
                ..
            }
        ));
    }

    #[test]
    fn a_full_ladder_of_collisions_fails_naming_the_last() {
        let mut fake = Fake {
            script: (0..5)
                .map(|_| Attempt::Collision("collision".into()))
                .collect(),
            ..Default::default()
        };
        let Outcome::Blocked(why) = go(&mut fake, &Deadline::new(Duration::from_secs(75))) else {
            panic!("expected a blocked spawn");
        };
        assert!(why.starts_with("5 attempt(s)"), "{why}");
    }

    #[test]
    fn another_refusal_is_not_lifted() {
        let mut fake = Fake {
            script: vec![Attempt::Refused("invalid description".into())],
            ..Default::default()
        };
        assert!(matches!(
            go(&mut fake, &Deadline::new(Duration::from_secs(75))),
            Outcome::Failed(_)
        ));
        assert_eq!(fake.spawned_at.len(), 1);
    }

    #[test]
    fn attempts_stop_when_the_request_runs_out_of_time() {
        // A slow server answering every attempt with a collision after ~29 s: the clock is
        // advanced by the fake per attempt.
        let t0 = Instant::now();
        let deadline = Deadline::starting_at(t0, Duration::from_secs(75));
        let clock = std::cell::Cell::new(t0);
        let mut fake = Fake {
            script: (0..5)
                .map(|_| Attempt::Collision("collision".into()))
                .collect(),
            ..Default::default()
        };
        struct Slow<'a>(&'a mut Fake, &'a std::cell::Cell<Instant>);
        impl SpawnBackend for Slow<'_> {
            fn spawn(&mut self, z: f32, timeout: Duration) -> Attempt {
                self.1
                    .set(self.1.get() + timeout.min(Duration::from_secs(29)));
                self.0.spawn(z, timeout)
            }
            fn find_late(&mut self, w: u32, t: Duration) -> Result<Vec<u32>, String> {
                self.0.find_late(w, t)
            }
        }
        let out = run(
            &mut Slow(&mut fake, &clock),
            HEIGHTS,
            9000,
            &deadline,
            RPC,
            || clock.get(),
        );
        let Outcome::Failed(why) = out else {
            panic!("expected out of time");
        };
        assert!(why.contains("out of time"), "{why}");
        // 75 s less the lookup and the settling reserve leaves 45 s: one full attempt, and
        // one of 16 s.
        assert_eq!(
            fake.timeouts,
            vec![Duration::from_secs(30), Duration::from_secs(16)]
        );
        assert!(
            clock.get() - t0 + Duration::from_secs(10) + ADOPT_LOOKUP_TIMEOUT
                < Duration::from_secs(90)
        );
    }
}
