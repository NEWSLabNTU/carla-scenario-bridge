//! Every time bound the bridge puts on CARLA, and how they nest inside SSv2's (roadmap 034
//! step 3, `docs/design/failure-and-frame-budget.md`, "Timeouts").
//!
//! SSv2's fork waits `SIMULATOR_RESPONSE_TIMEOUT` (90 s) for every request but `Initialize`,
//! which gets `SIMULATOR_INITIALIZE_TIMEOUT` (300 s). A request that runs past either is
//! lost: SSv2 recreates its socket and reports "No response from the simulator", and the
//! bridge's own error never arrives. So each bound below is chosen to deliver the bridge's
//! error -- after the state read that follows a hung call ([`HUNG_STATE_READ_WAIT`]) --
//! inside SSv2's, and the tests at the bottom check the arithmetic.

use std::time::{Duration, Instant};

/// SSv2's receive timeout for every request but `Initialize` (`zmq_multi_client.cpp`,
/// `SIMULATOR_RESPONSE_TIMEOUT`, since 2026-10-10).
#[cfg_attr(not(test), allow(dead_code))]
pub const SSV2_RESPONSE_TIMEOUT: Duration = Duration::from_secs(90);

/// SSv2's receive timeout for `Initialize` (`SIMULATOR_INITIALIZE_TIMEOUT`).
pub const SSV2_INITIALIZE_TIMEOUT: Duration = Duration::from_secs(300);

/// The CARLA client's RPC timeout (`Client::set_timeout`, `main.rs`): every RPC, including
/// a tick's `tick_cue`, a spawn and `set_episode_settings`.
pub const CLIENT_RPC_TIMEOUT: Duration = Duration::from_secs(30);

/// The RPC timeout around a map load (`load_world` and the map listing), which take longer
/// than any other call (66 s first load measured; >30 s listings on a fresh server).
pub const LOAD_WORLD_RPC_TIMEOUT: Duration = Duration::from_secs(120);

/// How long the state read after a hung call may take: a frame number from the client's
/// cache and one settings RPC, normally milliseconds; on its own thread.
pub const HUNG_STATE_READ_WAIT: Duration = Duration::from_secs(10);

/// The frame wait handed to LibCarla's `Tick` (carla-rust's `tick()` passes 60 s): 5x the
/// worst legitimate tick total measured (1.90 s over 1137 frame-budget windows since
/// 2026-10-11; p99 of the window maxima 0.69 s). Also the frame wait of every tick
/// `apply_settings` runs.
pub const LIBCARLA_TICK_TIMEOUT: Duration = Duration::from_secs(10);

/// The bound on one `world.tick()`. Above everything LibCarla bounds itself in a tick --
/// the `tick_cue` RPC ([`CLIENT_RPC_TIMEOUT`]) plus the frame wait
/// ([`LIBCARLA_TICK_TIMEOUT`]) -- so a timeout LibCarla reports stays the ordinary tick
/// error (the reconnect path), and only a wait LibCarla does not bound -- the Traffic
/// Manager's step handshake (034) -- trips it. With the state read after it, 55 s, inside
/// SSv2's 90.
pub const TICK_WATCHDOG: Duration = Duration::from_secs(45);

/// The bound on one `apply_settings`: `set_episode_settings` ([`CLIENT_RPC_TIMEOUT`]), a 1 s
/// frame sync, then ticks (each [`LIBCARLA_TICK_TIMEOUT`]) until two snapshots show the new
/// step -- normally two; measured 4.3 s on a loaded host. Each of those ticks runs the
/// Traffic Manager handshake too (`World.cpp:56-84`, `Simulator.cpp:282-285`).
pub const SETTINGS_WATCHDOG: Duration = Duration::from_secs(45);

/// The bound on one `load_world`: the load RPC ([`LOAD_WORLD_RPC_TIMEOUT`]) and then
/// LibCarla's wait for the new episode (50 ms polls with a `tick_cue` each in synchronous
/// mode, normally a few). Loads measured 4-66 s.
pub const LOAD_WORLD_WATCHDOG: Duration = Duration::from_secs(150);

/// The bound on a Traffic Manager call that joins or signals its worker (`ShutDown`,
/// `SetSynchronousMode`): both use the step handshake that can lose a wakeup (034).
/// Normally milliseconds; the worker's last cycle may make RPCs, so the tick's bound.
pub const TM_CALL_WATCHDOG: Duration = TICK_WATCHDOG;

/// Everything one `Initialize` does, from the request to the reply: the watched steps are
/// capped by what is left of it, and with the state read after a hung one it stays inside
/// SSv2's 300 s.
pub const INITIALIZE_BUDGET: Duration = Duration::from_secs(280);

/// Everything one spawn request does -- every attempt of the lift ladder, the adoption of a
/// late actor, the settling tick -- capped the same way inside SSv2's 90 s (034 step 4).
pub const SPAWN_REQUEST_BUDGET: Duration = Duration::from_secs(75);

/// Between scenarios, how long without a request before the bridge checks its CARLA
/// connection ([`crate::sim::SimBackend::probe_connection_idle`]), and how often.
/// Frames come every 50 ms during a scenario, so 5 s of quiet means none is running.
pub const IDLE_PROBE_AFTER: Duration = Duration::from_secs(5);
pub const IDLE_PROBE_EVERY: Duration = Duration::from_secs(2);

/// The RPC timeout of that check. The probe asks for the server version, which CARLA's RPC
/// threads answer without the game thread, so a live server answers in milliseconds even
/// while the world waits for a tick; only a dead connection runs into this bound.
pub const IDLE_PROBE_TIMEOUT: Duration = Duration::from_secs(2);

/// Consecutive failed checks before the connection counts as lost and the bridge exits.
pub const IDLE_PROBE_FAILURES: u32 = 2;

/// A request's deadline: what is left of its budget, and the bound each step gets.
#[derive(Debug, Clone, Copy)]
pub struct Deadline {
    start: Instant,
    budget: Duration,
}

impl Deadline {
    pub fn new(budget: Duration) -> Self {
        Self::starting_at(Instant::now(), budget)
    }

    pub fn starting_at(start: Instant, budget: Duration) -> Self {
        Self { start, budget }
    }

    pub fn remaining_at(&self, now: Instant) -> Duration {
        self.budget
            .saturating_sub(now.saturating_duration_since(self.start))
    }

    pub fn remaining(&self) -> Duration {
        self.remaining_at(Instant::now())
    }

    pub fn expired(&self) -> bool {
        self.remaining().is_zero()
    }

    /// The bound a step that may take `step` gets: its own, or what is left if less, but
    /// never under `floor` (a step given no time at all would only fail without trying).
    pub fn cap_at(&self, now: Instant, step: Duration, floor: Duration) -> Duration {
        step.min(self.remaining_at(now)).max(floor)
    }

    pub fn cap(&self, step: Duration, floor: Duration) -> Duration {
        self.cap_at(Instant::now(), step, floor)
    }

    pub fn budget(&self) -> Duration {
        self.budget
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tick_libcarla_bounds_itself_fails_before_the_watchdog() {
        // tick_cue under the client timeout, then the frame wait: LibCarla's own error.
        assert!(CLIENT_RPC_TIMEOUT + LIBCARLA_TICK_TIMEOUT < TICK_WATCHDOG);
        // And far above the slowest legitimate tick (1.90 s).
        assert!(LIBCARLA_TICK_TIMEOUT >= Duration::from_millis(1900) * 5);
    }

    #[test]
    fn a_hung_tick_or_settings_change_is_reported_inside_ssv2s_wait() {
        for bound in [TICK_WATCHDOG, SETTINGS_WATCHDOG, TM_CALL_WATCHDOG] {
            assert!(bound + HUNG_STATE_READ_WAIT <= Duration::from_secs(55));
            assert!(bound + HUNG_STATE_READ_WAIT < SSV2_RESPONSE_TIMEOUT);
        }
        assert!(
            CLIENT_RPC_TIMEOUT + Duration::from_secs(1) + LIBCARLA_TICK_TIMEOUT
                <= SETTINGS_WATCHDOG
        );
    }

    #[test]
    fn a_map_load_is_watched_above_its_own_rpc_timeout() {
        assert!(LOAD_WORLD_RPC_TIMEOUT < LOAD_WORLD_WATCHDOG);
        assert!(LOAD_WORLD_WATCHDOG + HUNG_STATE_READ_WAIT < INITIALIZE_BUDGET);
    }

    #[test]
    fn whole_requests_answer_inside_ssv2s_timeouts() {
        assert!(INITIALIZE_BUDGET + HUNG_STATE_READ_WAIT < SSV2_INITIALIZE_TIMEOUT);
        assert!(SPAWN_REQUEST_BUDGET + HUNG_STATE_READ_WAIT < SSV2_RESPONSE_TIMEOUT);
    }

    #[test]
    fn a_deadline_caps_each_step_at_what_is_left() {
        let t0 = Instant::now();
        let d = Deadline::starting_at(t0, Duration::from_secs(100));
        let s = Duration::from_secs;
        assert_eq!(d.remaining_at(t0), s(100));
        assert_eq!(d.remaining_at(t0 + s(30)), s(70));
        assert_eq!(d.remaining_at(t0 + s(130)), Duration::ZERO);
        assert_eq!(d.cap_at(t0, s(45), s(10)), s(45));
        assert_eq!(d.cap_at(t0 + s(80), s(45), s(10)), s(20));
        // Past the end a step still gets its floor, not nothing.
        assert_eq!(d.cap_at(t0 + s(99), s(45), s(10)), s(10));
    }

    #[test]
    fn initializes_steps_at_their_bounds_fit_its_budget() {
        // Initialize's blocking steps in order, each taking all it is allowed; a step past
        // the deadline is not started (Initialize fails with the step's name instead).
        let steps = [
            ("settings probe", CLIENT_RPC_TIMEOUT),
            ("synchronous mode", SETTINGS_WATCHDOG),
            ("Traffic Manager shutdown", TM_CALL_WATCHDOG),
            ("map listing", LOAD_WORLD_RPC_TIMEOUT),
            ("map load", LOAD_WORLD_WATCHDOG),
            ("synchronous mode in the new world", SETTINGS_WATCHDOG),
            ("reaper census", CLIENT_RPC_TIMEOUT),
        ];
        let t0 = Instant::now();
        let d = Deadline::starting_at(t0, INITIALIZE_BUDGET);
        let mut now = t0;
        for (_, bound) in steps {
            if d.remaining_at(now).is_zero() {
                break;
            }
            now += d.cap_at(now, bound, Duration::from_secs(1));
        }
        let worst = now - t0 + HUNG_STATE_READ_WAIT;
        assert!(worst < SSV2_INITIALIZE_TIMEOUT, "{worst:?}");
    }
}

/// How long the bridge's idle check lets its world server go unanswered before it gives up
/// and exits for a fresh bridge (roadmap 046). Long enough for a supervised world server to
/// be respawned (CARLA's after it lost its CARLA, seconds), so a restart there does not
/// restart the bridge too; the bridge finds the new instance and forgets the old one's
/// entities.
pub const WORLD_SERVER_LOST_AFTER: Duration = Duration::from_secs(60);
