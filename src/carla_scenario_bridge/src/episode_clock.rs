//! Simulation time as CARLA reports it, continued across world reloads (roadmap 015).
//!
//! `sim_time = elapsed_seconds + epoch`. CARLA restarts `elapsed_seconds` at 0 for every
//! episode, and Autoware has no answer to a clock that goes backwards, so an episode change
//! is turned into a pause: whoever sees it sets `epoch += E_last + Δ_last` (the old
//! episode's last frame time and step), and the new episode's first frame lands one step
//! after the old one's last.
//!
//! acb applies the same rule on its own, from the same frame stream, with no channel between
//! the two. They agree bit-exactly only if they saw the same last frame. That is why
//! [`reload_as_pause`] switches CARLA to synchronous mode *before* it reads the last
//! snapshot: in sync mode nothing ticks unless csb ticks, so the frame csb reads is the last
//! frame acb's `on_tick` receives before the reload.

/// The episode epoch, and the last CARLA frame time csb has read.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct EpisodeClock {
    /// Seconds added to CARLA's `elapsed_seconds`. Starts at 0.
    epoch: f64,
    /// `(elapsed_seconds, delta_seconds)` of the newest snapshot read, if any. The fallback
    /// for an episode change whose pre-reload snapshot could not be read.
    last_frame: Option<(f64, f64)>,
}

/// What an episode change did to the epoch, for the log line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EpisodeChange {
    pub old_epoch: f64,
    pub new_epoch: f64,
    /// Simulation time of the new episode's first frame (`elapsed_seconds` = 0).
    pub continues_at: f64,
}

impl EpisodeClock {
    #[cfg(test)]
    pub fn epoch(&self) -> f64 {
        self.epoch
    }

    /// The simulation time of a frame CARLA reports as `elapsed` seconds into its episode,
    /// remembered as the latest frame seen.
    pub fn observe(&mut self, elapsed: f64, delta: f64) -> f64 {
        self.last_frame = Some((elapsed, delta));
        self.sim_time(elapsed)
    }

    pub fn sim_time(&self, elapsed: f64) -> f64 {
        elapsed + self.epoch
    }

    pub fn last_frame(&self) -> Option<(f64, f64)> {
        self.last_frame
    }

    /// Apply the episode rule: the old episode ended at `last_elapsed` with step
    /// `last_delta`, so the new one starts one step later.
    pub fn begin_episode(&mut self, last_elapsed: f64, last_delta: f64) -> EpisodeChange {
        let old_epoch = self.epoch;
        self.epoch += last_elapsed + last_delta;
        // The new episode's frames are numbered from 0 again; the old one's last frame is
        // no fallback for the next change.
        self.last_frame = None;
        EpisodeChange {
            old_epoch,
            new_epoch: self.epoch,
            continues_at: self.sim_time(0.0),
        }
    }
}

/// The CARLA operations an episode change is made of, so their order can be tested
/// without a server.
pub trait EpisodeReload {
    type World;
    type Error;
    /// Put CARLA in synchronous mode, so nothing ticks until csb does.
    fn enter_sync(&mut self) -> Result<(), Self::Error>;
    /// `(elapsed_seconds, delta_seconds)` of the current snapshot.
    fn read_frame(&mut self) -> Result<(f64, f64), Self::Error>;
    /// Load the new world.
    fn reload(&mut self) -> Result<Self::World, Self::Error>;
}

/// Outcome of [`reload_as_pause`].
pub struct PausedReload<W, E> {
    /// The loaded world, or why loading failed.
    pub world: Result<W, E>,
    /// The old episode's last frame, if it could be read. `None` falls back to the last
    /// frame the caller saw (see [`EpisodeClock::last_frame`]).
    pub last_frame: Option<(f64, f64)>,
    /// Why sync mode could not be entered, if it could not. The reload still happens -- the
    /// scenario needs its town -- but acb may then have seen a later last frame than csb.
    pub sync_error: Option<E>,
    /// Why the last frame could not be read, if it could not.
    pub read_error: Option<E>,
}

/// Enter sync mode, read the last frame, then reload -- in that order, nothing between.
pub fn reload_as_pause<B: EpisodeReload>(backend: &mut B) -> PausedReload<B::World, B::Error> {
    let sync_error = backend.enter_sync().err();
    let (last_frame, read_error) = match backend.read_frame() {
        Ok(frame) => (Some(frame), None),
        Err(e) => (None, Some(e)),
    };
    let world = backend.reload();
    PausedReload {
        world,
        last_frame,
        sync_error,
        read_error,
    }
}

/// Run `ticks` ticks (at least one), then read the frame -- the frame SSv2 is told about is
/// the one after the frame's last tick. Stops at the first failed tick.
pub fn tick_then_read<W, E, R>(
    world: &mut W,
    ticks: u32,
    mut tick: impl FnMut(&mut W) -> Result<(), E>,
    read: impl FnOnce(&W) -> R,
) -> Result<R, E> {
    for _ in 0..ticks.max(1) {
        tick(world)?;
    }
    Ok(read(world))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A CARLA stand-in: f32 step, as CARLA's `fixed_delta_seconds` is, accumulated in f64.
    struct FakeCarla {
        elapsed: f64,
        delta: f64,
        sync: bool,
        log: Vec<&'static str>,
    }

    impl FakeCarla {
        fn new(step: f32) -> Self {
            Self {
                elapsed: 0.0,
                delta: step as f64,
                sync: false,
                log: Vec::new(),
            }
        }
        fn tick(&mut self) {
            self.elapsed += self.delta;
        }
    }

    impl EpisodeReload for FakeCarla {
        type World = &'static str;
        type Error = String;
        fn enter_sync(&mut self) -> Result<(), String> {
            self.sync = true;
            self.log.push("sync");
            Ok(())
        }
        fn read_frame(&mut self) -> Result<(f64, f64), String> {
            self.log.push(if self.sync {
                "read(sync)"
            } else {
                "read(async)"
            });
            Ok((self.elapsed, self.delta))
        }
        fn reload(&mut self) -> Result<&'static str, String> {
            self.log.push("reload");
            self.elapsed = 0.0;
            Ok("Town02")
        }
    }

    #[test]
    fn response_time_is_the_snapshot_after_the_last_tick_plus_the_epoch() {
        for substeps in [1_u32, 2, 4] {
            let mut carla = FakeCarla::new(0.05 / substeps as f32);
            let mut clock = EpisodeClock::default();
            clock.begin_episode(123.45, 0.05);
            let mut expected_elapsed = 0.0;
            for _ in 0..100 {
                let (elapsed, delta) = tick_then_read(
                    &mut carla,
                    substeps,
                    |c| -> Result<(), ()> {
                        c.tick();
                        Ok(())
                    },
                    |c| (c.elapsed, c.delta),
                )
                .unwrap();
                for _ in 0..substeps {
                    expected_elapsed += (0.05 / substeps as f32) as f64;
                }
                assert_eq!(elapsed, expected_elapsed, "read after all {substeps} ticks");
                assert_eq!(
                    clock.observe(elapsed, delta),
                    expected_elapsed + clock.epoch()
                );
            }
        }
    }

    #[test]
    fn a_failed_tick_skips_the_rest_and_the_read() {
        let mut ticks = 0_u32;
        let mut read = false;
        let out = tick_then_read(
            &mut ticks,
            4,
            |t| {
                *t += 1;
                if *t == 2 {
                    Err("boom")
                } else {
                    Ok(())
                }
            },
            |_| read = true,
        );
        assert_eq!(out, Err("boom"));
        assert_eq!(ticks, 2);
        assert!(!read);
    }

    #[test]
    fn zero_substeps_still_ticks_once() {
        let mut ticks = 0_u32;
        let _ = tick_then_read(
            &mut ticks,
            0,
            |t| -> Result<(), ()> {
                *t += 1;
                Ok(())
            },
            |_| (),
        );
        assert_eq!(ticks, 1);
    }

    #[test]
    fn epoch_starts_at_zero() {
        let mut clock = EpisodeClock::default();
        assert_eq!(clock.observe(812.5, 0.05), 812.5);
    }

    #[test]
    fn a_reload_continues_one_step_after_the_last_frame() {
        let mut carla = FakeCarla::new(0.05);
        let mut clock = EpisodeClock::default();
        let mut last_sim = 0.0;
        for _ in 0..1000 {
            carla.tick();
            last_sim = clock.observe(carla.elapsed, carla.delta);
        }

        let out = reload_as_pause(&mut carla);
        assert_eq!(out.world, Ok("Town02"));
        let (e_last, d_last) = out.last_frame.unwrap();
        let change = clock.begin_episode(e_last, d_last);

        assert_eq!(change.old_epoch, 0.0);
        assert_eq!(change.new_epoch, e_last + d_last);
        // New episode, elapsed back at 0: the clock lands exactly one step after the last.
        assert_eq!(change.continues_at, last_sim + carla.delta);
        assert_eq!(clock.sim_time(0.0), last_sim + carla.delta);
        carla.tick();
        assert!(clock.observe(carla.elapsed, carla.delta) > last_sim + carla.delta);
    }

    #[test]
    fn epochs_accumulate_over_several_reloads() {
        let mut clock = EpisodeClock::default();
        clock.begin_episode(10.0, 0.05);
        let second = clock.begin_episode(20.0, 0.05);
        assert_eq!(second.old_epoch, 10.05);
        assert_eq!(second.new_epoch, 10.05 + 20.05);
        assert_eq!(clock.last_frame(), None);
    }

    #[test]
    fn sync_mode_is_on_before_the_pre_reload_snapshot() {
        let mut carla = FakeCarla::new(0.05);
        let out = reload_as_pause(&mut carla);
        assert!(out.sync_error.is_none() && out.read_error.is_none());
        assert_eq!(carla.log, ["sync", "read(sync)", "reload"]);
    }

    /// A backend whose sync switch fails: the reload still happens, and says why the epoch
    /// may disagree with acb's.
    #[test]
    fn a_failed_sync_switch_still_reloads_and_reports_it() {
        struct NoSync(FakeCarla);
        impl EpisodeReload for NoSync {
            type World = &'static str;
            type Error = String;
            fn enter_sync(&mut self) -> Result<(), String> {
                Err("refused".into())
            }
            fn read_frame(&mut self) -> Result<(f64, f64), String> {
                self.0.read_frame()
            }
            fn reload(&mut self) -> Result<&'static str, String> {
                self.0.reload()
            }
        }
        let mut b = NoSync(FakeCarla::new(0.05));
        let out = reload_as_pause(&mut b);
        assert_eq!(out.sync_error.as_deref(), Some("refused"));
        assert!(out.last_frame.is_some());
        assert_eq!(out.world, Ok("Town02"));
    }
}
