//! A bounded wait on every `world.tick()` (roadmap 032 N5).
//!
//! # Why
//!
//! In step 7 the bridge hung for 12+ minutes inside the settling tick after spawning
//! `simulator_autopilot`'s vehicle, right after it created a synchronous Traffic Manager,
//! while CARLA kept answering RPC: no line logged, every thread in a futex or epoll wait, SSv2
//! left to its own 420 s receive timeout, and the next scenario failed the same way. LibCarla
//! bounds only part of a tick -- `tick_cue` is an RPC under the client timeout (30 s), and the
//! wait for the new frame is carla-rust's 60 s -- and neither fired, so the wait was in a
//! part with no bound at all (in 0.9.16 a synchronous Traffic Manager steps inside
//! `World::Tick` and hands its step over on a condition variable with no timeout). A longer or
//! shorter client timeout (`set_timeout`) therefore changes nothing.
//!
//! # How
//!
//! The tick runs on one long-lived helper thread; the ZMQ thread hands it the call and waits
//! at most [`TICK_WATCHDOG`]. Ticks stay strictly one at a time, issued only by the ZMQ
//! thread's request and awaited before the request goes on, so csb is still CARLA's only
//! ticker and no request overlaps a tick (015/023's synchronous-mode invariants). On expiry
//! the request fails with an error naming the step and the wait, and the watchdog is wedged:
//! the helper may still be inside LibCarla holding the world, so no further tick is ever
//! issued from this process. The coordinator then ends the bridge after replying, and its
//! supervisor (`respawn`) starts a fresh one -- the fresh bridge passed in step 7.

use std::sync::mpsc;
use std::time::{Duration, Instant};

/// The bound on one `world.tick()`. The slowest legitimate ticks measured: an `UpdateFrame`'s
/// ticks (all substeps) at most 1.90 s over 1137 frame-budget windows of every bridge log
/// since 2026-10-11 (p99 of the window maxima 0.69 s), spawn frames with their settling tick
/// 0.30-0.64 s (032 steps 4-5); a map load is not a tick (`load_world`, under its own 120 s
/// client timeout). 90 s is 47x the worst of them, and above carla-rust's own 60 s frame wait
/// so that a timeout LibCarla does report stays the ordinary tick error (the reconnect
/// path); with the state read after expiry it leaves SSv2's 420 s receive timeout room to
/// deliver the error.
pub const TICK_WATCHDOG: Duration = Duration::from_secs(90);

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

type Job<T> = Box<dyn FnOnce() -> T + Send>;
type Outcome<T> = std::thread::Result<T>;
/// The helper's two ends: calls in, outcomes back.
type Worker<T> = (mpsc::Sender<Job<T>>, mpsc::Receiver<Outcome<T>>);

/// Runs calls on one helper thread with a bounded wait for each.
pub struct Watchdog<T> {
    worker: Option<Worker<T>>,
    wedged: bool,
}

impl<T: Send + 'static> Default for Watchdog<T> {
    fn default() -> Self {
        Self {
            worker: None,
            wedged: false,
        }
    }
}

impl<T: Send + 'static> Watchdog<T> {
    /// Whether an earlier call expired: nothing more will run.
    #[cfg(test)]
    pub fn is_wedged(&self) -> bool {
        self.wedged
    }

    /// Run `job` on the helper thread and wait at most `bound` for it. A panic in `job` is
    /// resumed here, on the caller's thread, and the helper keeps serving.
    pub fn run(
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
        if jobs.send(Box::new(job)).is_err() {
            self.worker = None;
            return Err(WatchError::WorkerGone);
        }
        let started = Instant::now();
        match results.recv_timeout(bound) {
            Ok(Ok(value)) => Ok(value),
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

fn spawn_worker<T: Send + 'static>() -> Option<Worker<T>> {
    let (job_tx, job_rx) = mpsc::channel::<Job<T>>();
    let (out_tx, out_rx) = mpsc::channel::<Outcome<T>>();
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
        let mut numbers = Watchdog::default();
        assert_eq!(numbers.run(Duration::from_secs(5), || 7), Ok(7));
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
        let mut w = Watchdog::<u32>::default();
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            w.run(Duration::from_secs(5), || panic!("boom"))
        }));
        assert!(r.is_err());
        assert_eq!(w.run(Duration::from_secs(5), || 3), Ok(3));
    }
}
