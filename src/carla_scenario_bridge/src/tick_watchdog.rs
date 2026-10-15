//! A bounded wait on every `world.tick()` (roadmap 032 N5).
//!
//! # Why
//!
//! In step 7 the bridge hung for 12+ minutes inside the settling tick after spawning
//! `simulator_autopilot`'s vehicle, right after it created a synchronous Traffic Manager,
//! while CARLA kept answering RPC: no line logged, every thread in a futex or epoll wait, SSv2
//! left to its own receive timeout, and the next scenario failed the same way. LibCarla
//! bounds only part of a tick -- `tick_cue` is an RPC under the client timeout (30 s), and the
//! wait for the new frame is the timeout passed to `Tick` (then carla-rust's default 60 s) --
//! and neither fired, so the wait was in a
//! part with no bound at all: in LibCarla 0.9.16 `SynchronizeFrame` (inside `World::Tick`),
//! once the frame has arrived, calls `TrafficManager::Tick`, whose `TrafficManagerLocal::
//! SynchronousTick` waits on `step_end_trigger` with no timeout. A longer or
//! shorter client timeout (`set_timeout`) therefore changes nothing.
//!
//! # How
//!
//! The tick runs on one long-lived helper thread; the ZMQ thread hands it the call and waits
//! at most its bound (`crate::budgets`: one per kind of call). Ticks stay strictly one at a time, issued only by the ZMQ
//! thread's request and awaited before the request goes on, so csb is still CARLA's only
//! ticker and no request overlaps a tick (015/023's synchronous-mode invariants). On expiry
//! the request fails with an error naming the step and the wait, and the watchdog is wedged:
//! the helper may still be inside LibCarla holding the world, so no further tick is ever
//! issued from this process. The coordinator then ends the bridge after replying, and its
//! supervisor (`respawn`) starts a fresh one -- the fresh bridge passed in step 7.

use std::any::Any;
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// A test-only fault (roadmap 034 step 2): the tick of one SSv2 frame never returns, as in
/// the step-7 hang, so the drain-and-exit path can be run on a live stack.
///
/// Off unless the environment sets `CSB_TEST_HANG_TICK_AT=<SSv2 frame>`: the first tick of
/// that `UpdateFrame` blocks its helper thread forever instead of calling CARLA. With
/// `CSB_TEST_HANG_TICK_TOKEN=<file>` it fires only while that file exists, and removes it
/// when it fires, so one `touch` arms exactly one hang and the respawned bridge (which
/// inherits the environment) runs the next scenario normally. Never set in a product
/// launch; the bridge logs a warning at start-up when it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HangHook {
    frame: u64,
    token: Option<std::path::PathBuf>,
}

impl HangHook {
    pub const FRAME_ENV: &'static str = "CSB_TEST_HANG_TICK_AT";
    pub const TOKEN_ENV: &'static str = "CSB_TEST_HANG_TICK_TOKEN";

    /// The hook the environment asks for, if any.
    pub fn from_env() -> Option<Self> {
        let at = std::env::var(Self::FRAME_ENV).ok();
        let token = std::env::var(Self::TOKEN_ENV).ok();
        Self::parse(at.as_deref(), token.as_deref())
    }

    pub fn parse(at: Option<&str>, token: Option<&str>) -> Option<Self> {
        let frame = at?.trim().parse().ok()?;
        let token = token
            .map(str::trim)
            .filter(|t| !t.is_empty())
            .map(std::path::PathBuf::from);
        Some(Self { frame, token })
    }

    /// Whether the tick of SSv2 frame `frame` is to hang; consumes the token if it is.
    pub fn fires_at(&self, frame: u64) -> bool {
        if frame != self.frame {
            return false;
        }
        match &self.token {
            None => true,
            Some(token) => std::fs::remove_file(token).is_ok(),
        }
    }

    pub fn describe(&self) -> String {
        match &self.token {
            None => format!(
                "the first tick of SSv2 frame {} hangs, every time",
                self.frame
            ),
            Some(t) => format!(
                "the first tick of SSv2 frame {} hangs while {} exists (once per touch)",
                self.frame,
                t.display()
            ),
        }
    }
}

/// Why a watched call did not return a result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchError {
    /// The call did not return within the bound; `waited` is how long it was given.
    Expired { waited: Duration },
    /// An earlier call never returned: nothing more is run.
    Wedged,
    /// The helper thread could not be started or has gone.
    WorkerGone,
}

type Value = Box<dyn Any + Send>;
type Job = Box<dyn FnOnce() -> Value + Send>;
type Outcome = std::thread::Result<Value>;
/// The helper's two ends: calls in, outcomes back.
type Worker = (mpsc::Sender<Job>, mpsc::Receiver<Outcome>);

/// Runs calls on one helper thread with a bounded wait for each. Any call that can block in
/// LibCarla goes through the same helper -- ticks, settings changes, map loads, Traffic
/// Manager calls (034 step 3) -- so they stay strictly one at a time, and one that never
/// returns stops them all.
#[derive(Default)]
pub struct Watchdog {
    worker: Option<Worker>,
    wedged: bool,
}

impl Watchdog {
    /// Whether an earlier call expired: nothing more will run.
    #[cfg(test)]
    pub fn is_wedged(&self) -> bool {
        self.wedged
    }

    /// Run `job` on the helper thread and wait at most `bound` for it. A panic in `job` is
    /// resumed here, on the caller's thread, and the helper keeps serving.
    pub fn run<T: Send + 'static>(
        &mut self,
        bound: Duration,
        job: impl FnOnce() -> T + Send + 'static,
    ) -> Result<T, WatchError> {
        if self.wedged {
            return Err(WatchError::Wedged);
        }
        if self.worker.is_none() {
            self.worker = Some(spawn_worker().ok_or(WatchError::WorkerGone)?);
        }
        let (jobs, results) = self.worker.as_ref().expect("started above");
        if jobs
            .send(Box::new(move || Box::new(job()) as Value))
            .is_err()
        {
            self.worker = None;
            return Err(WatchError::WorkerGone);
        }
        let started = Instant::now();
        match results.recv_timeout(bound) {
            Ok(Ok(value)) => Ok(*value
                .downcast::<T>()
                .expect("the helper returns what its job returned")),
            Ok(Err(panic)) => std::panic::resume_unwind(panic),
            Err(mpsc::RecvTimeoutError::Timeout) => {
                self.wedged = true;
                Err(WatchError::Expired {
                    waited: started.elapsed(),
                })
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                self.worker = None;
                Err(WatchError::WorkerGone)
            }
        }
    }
}

fn spawn_worker() -> Option<Worker> {
    let (job_tx, job_rx) = mpsc::channel::<Job>();
    let (out_tx, out_rx) = mpsc::channel::<Outcome>();
    std::thread::Builder::new()
        .name("csb-tick".into())
        .spawn(move || {
            for job in job_rx {
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(job));
                if out_tx.send(outcome).is_err() {
                    break;
                }
            }
        })
        .ok()?;
    Some((job_tx, out_rx))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_hang_hook_is_off_unless_set_to_a_frame() {
        assert_eq!(HangHook::parse(None, None), None);
        assert_eq!(HangHook::parse(Some(""), None), None);
        assert_eq!(HangHook::parse(Some("soon"), None), None);
        let hook = HangHook::parse(Some(" 40 "), None).unwrap();
        assert!(!hook.fires_at(39));
        assert!(hook.fires_at(40));
        // No token: every bridge process that reaches the frame hangs there.
        assert!(hook.fires_at(40));
    }

    #[test]
    fn the_hang_hook_with_a_token_fires_once() {
        let dir = std::env::temp_dir().join(format!("csb-hang-hook-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let token = dir.join("token");
        let hook = HangHook::parse(Some("5"), Some(token.to_str().unwrap())).unwrap();
        // Disarmed until the token exists.
        assert!(!hook.fires_at(5));
        std::fs::write(&token, b"").unwrap();
        assert!(!hook.fires_at(4));
        assert!(token.exists(), "only the frame consumes it");
        assert!(hook.fires_at(5));
        assert!(!token.exists());
        // The respawned bridge runs the next scenario through frame 5.
        assert!(!hook.fires_at(5));
        let _ = std::fs::remove_dir_all(&dir);
    }

    const SHORT: Duration = Duration::from_millis(100);

    #[test]
    fn results_pass_through_on_one_helper_thread() {
        let mut w = Watchdog::default();
        let first = w
            .run(Duration::from_secs(5), || std::thread::current().id())
            .unwrap();
        let second = w
            .run(Duration::from_secs(5), || std::thread::current().id())
            .unwrap();
        assert_eq!(
            first, second,
            "one long-lived helper, not a thread per tick"
        );
        assert_ne!(first, std::thread::current().id());
        // One helper serves calls of any result type, one at a time.
        assert_eq!(w.run(Duration::from_secs(5), || 7), Ok(7));
        assert_eq!(
            w.run(Duration::from_secs(5), || "settings".to_string()),
            Ok("settings".to_string())
        );
        assert!(!w.is_wedged());
    }

    #[test]
    fn a_call_past_the_bound_fails_and_wedges_the_watchdog() {
        let mut w = Watchdog::default();
        let (release, held) = mpsc::channel::<()>();
        let started = Instant::now();
        // A tick that never returns on its own (the step-7 hang), released at the end.
        let r = w.run(SHORT, move || {
            let _ = held.recv();
            1
        });
        let Err(WatchError::Expired { waited }) = r else {
            panic!("expected expiry, got {r:?}");
        };
        assert!(waited >= SHORT && started.elapsed() < Duration::from_secs(5));
        assert!(w.is_wedged());
        // The helper may still hold the world: nothing more runs, and nothing waits.
        let started = Instant::now();
        assert_eq!(w.run(SHORT, || 2), Err(WatchError::Wedged));
        assert!(started.elapsed() < SHORT);
        drop(release);
    }

    #[test]
    fn a_panic_reaches_the_caller_and_the_helper_survives() {
        let mut w = Watchdog::default();
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            w.run(Duration::from_secs(5), || -> u32 { panic!("boom") })
        }));
        assert!(r.is_err());
        assert_eq!(w.run(Duration::from_secs(5), || 3), Ok(3));
    }
}
