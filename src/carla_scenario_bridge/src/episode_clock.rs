//! Simulation time as CARLA reports it, continued across world reloads (roadmap 015).
//!
//! `sim_time = elapsed_seconds + epoch`. CARLA restarts `elapsed_seconds` at 0 for every
//! episode, and Autoware has no answer to a clock that goes backwards, so an episode change
//! is turned into a pause: whoever sees it sets `epoch += E_last + Δ_last` (the old
//! episode's last frame time and step), and the new episode's first frame lands one step
//! after the old one's last.
//!
//! All of it is integer nanoseconds, with one conversion, [`nanos`]: round half away from
//! zero of `seconds * 1e9`, applied to each term on its own --
//! `sim_ns = nanos(elapsed) + epoch_ns`, `epoch_ns += nanos(E_last) + nanos(Δ_last)`.
//! acb's `clock.rs` applies the identical rule, so the time csb reports to SSv2 and the
//! `/clock` acb publishes for the same frame are the same integer, not two roundings of an
//! f64 sum that disagree by 1 ns on a share of frames.
//!
//! acb applies the rule on its own, from the same frame stream, with no channel between
//! the two. They agree bit-exactly only if they saw the same last frame. That is why
//! [`reload_as_pause`] switches CARLA to synchronous mode *before* it reads the last
//! snapshot: in sync mode nothing ticks unless csb ticks, so the frame csb reads is the last
//! frame acb's `on_tick` receives before the reload.

/// Seconds to integer nanoseconds: `round(secs * 1e9)`, half away from zero (`f64::round`).
/// Must stay identical to acb's `clock::nanos`; both carry the same table of cases.
pub fn nanos(secs: f64) -> i64 {
    (secs * 1e9).round() as i64
}

/// The episode epoch, and the last CARLA frame time csb has read.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct EpisodeClock {
    /// Nanoseconds added to CARLA's `elapsed_seconds`. Starts at 0.
    epoch_ns: i64,
    /// `(elapsed_seconds, delta_seconds)` of the newest snapshot read, if any. The fallback
    /// for an episode change whose pre-reload snapshot could not be read.
    last_frame: Option<(f64, f64)>,
}

/// What an episode change did to the epoch, for the log line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EpisodeChange {
    pub old_epoch_ns: i64,
    pub new_epoch_ns: i64,
    /// Simulation time of the new episode's first frame (`elapsed_seconds` = 0), in ns.
    pub continues_at_ns: i64,
}

impl EpisodeClock {
    #[cfg(test)]
    pub fn epoch_ns(&self) -> i64 {
        self.epoch()
    }

    /// The simulation time (ns) of a frame CARLA reports as `elapsed` seconds into its
    /// episode, remembered as the latest frame seen.
    pub fn observe(&mut self, elapsed: f64, delta: f64) -> i64 {
        self.last_frame = Some((elapsed, delta));
        self.sim_ns(elapsed)
    }

    pub fn sim_ns(&self, elapsed: f64) -> i64 {
        nanos(elapsed) + self.epoch_ns
    }

    pub fn last_frame(&self) -> Option<(f64, f64)> {
        self.last_frame
    }

    /// A clock carried over from a previous csb process (see `clock_store`).
    pub fn restored(epoch_ns: i64, last_frame: Option<(f64, f64)>) -> Self {
        Self {
            epoch_ns,
            last_frame,
        }
    }

    pub fn epoch(&self) -> i64 {
        self.epoch_ns
    }

    /// Apply the episode rule: the old episode ended at `last_elapsed` with step
    /// `last_delta`, so the new one starts one step later.
    pub fn begin_episode(&mut self, last_elapsed: f64, last_delta: f64) -> EpisodeChange {
        let old_epoch_ns = self.epoch_ns;
        self.epoch_ns += nanos(last_elapsed) + nanos(last_delta);
        // The new episode's frames are numbered from 0 again; the old one's last frame is
        // no fallback for the next change.
        self.last_frame = None;
        EpisodeChange {
            old_epoch_ns,
            new_epoch_ns: self.epoch_ns,
            continues_at_ns: self.sim_ns(0.0),
        }
    }
}

/// Nanoseconds as `s.nnnnnnnnn` for log lines, without going through an f64.
pub fn fmt_ns(ns: i64) -> String {
    let sign = if ns < 0 { "-" } else { "" };
    let a = ns.unsigned_abs();
    format!("{sign}{}.{:09}", a / 1_000_000_000, a % 1_000_000_000)
}

/// The old episode's last `elapsed_seconds`, counted from csb's two snapshots around a
/// synchronous `load_world(town, reset_settings = false)` (roadmap 015 step 7).
///
/// CARLA's `LoadEpisode` ticks the old episode itself while it waits, so the frame csb read
/// before the load (`f0`, `e0`, step `delta`) is not the old episode's last: acb, following
/// `on_tick`, sees the extra frames. The frame counter continues across episodes, and the new
/// world (kept synchronous) only advances by `delta` per tick, so its snapshot `(f1, e1)`
/// says how many of the frames since `f0` belong to it -- `round(e1 / delta)` -- and the rest
/// were the old episode's. The old episode's elapsed time is then `e0` plus `delta` added
/// once per extra frame, accumulated as CARLA accumulates it so the result is bit-identical
/// to the frame acb saw last.
///
/// `None` if the numbers do not describe such a load (no step, frames going backwards, or an
/// implausible count): the caller then falls back to `e0`.
pub fn old_episode_last_elapsed(e0: f64, delta: f64, f0: u64, f1: u64, e1: f64) -> Option<f64> {
    if delta.is_nan() || delta <= 0.0 || f1 <= f0 || e1.is_nan() || e1 < 0.0 {
        return None;
    }
    let n_new = (e1 / delta).round();
    if !(0.0..=1e6).contains(&n_new) {
        return None;
    }
    let k_old = (f1 - f0) as i64 - n_new as i64;
    if !(0..=1000).contains(&k_old) {
        return None;
    }
    let mut e = e0;
    for _ in 0..k_old {
        e += delta;
    }
    Some(e)
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

    /// Measured live, 0.9.16 (2026-10-05): one old-episode frame after csb's snapshot, and
    /// the new world's first frame at `elapsed = delta`, both directions.
    #[test]
    fn frame_count_reproduces_the_frame_acbs_stream_ended_on() {
        let d = 0.05_f32 as f64;
        // Town01 -> Town02: F0 769975, E0 13.738989307, F1 769977, E1 0.050000001.
        let e0 = 13.738989307_f64;
        let got = old_episode_last_elapsed(e0, d, 769_975, 769_977, d).unwrap();
        assert_eq!(
            got,
            e0 + d,
            "k_old = 1: the stream's last old frame was F0 + 1"
        );
        // Synthetic: k_old extra old frames and n_new new ones.
        for k_old in 0..=5_u64 {
            for n_new in 1..=3_u64 {
                let mut e1 = 0.0;
                for _ in 0..n_new {
                    e1 += d;
                }
                let mut want = 100.0_f64;
                for _ in 0..k_old {
                    want += d;
                }
                let got = old_episode_last_elapsed(100.0, d, 1000, 1000 + k_old + n_new, e1);
                assert_eq!(got, Some(want), "k_old {k_old}, n_new {n_new}");
            }
        }
    }

    #[test]
    fn frame_count_refuses_numbers_that_are_not_a_synchronous_load() {
        assert_eq!(
            old_episode_last_elapsed(1.0, 0.0, 10, 12, 0.05),
            None,
            "no step"
        );
        assert_eq!(
            old_episode_last_elapsed(1.0, 0.05, 12, 10, 0.05),
            None,
            "frames backwards"
        );
        assert_eq!(
            old_episode_last_elapsed(1.0, 0.05, 10, 12, 0.5),
            None,
            "more new frames than frames"
        );
        assert_eq!(
            old_episode_last_elapsed(1.0, 0.05, 10, 5000, 0.05),
            None,
            "implausible count"
        );
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
                    nanos(expected_elapsed) + clock.epoch_ns()
                );
            }
        }
    }

    /// The shared table: acb's `clock.rs` carries the same cases and must give the same
    /// nanoseconds. Includes exact halves (half away from zero, not half to even) and
    /// CARLA's f32 step 0.050000000745 accumulated in f64, as the server does.
    pub(crate) const NANOS_CASES: &[(f64, i64)] = &[
        (0.0, 0),
        (0.05000000074505806, 50_000_001),
        (5e-10, 1),
        (1.5e-9, 2),
        (2.5e-9, 3),
        (-2.5e-9, -3),
        (3.5e-9, 4),
        (0.10000000149011612, 100_000_001),
        (0.15000000223517418, 150_000_002),
        (67.0000009983778, 67_000_000_998),
        (171.60000255703926, 171_600_002_557),
        (1000.0000149011612, 1_000_000_014_901),
        (237000.05000000075, 237_000_050_000_001),
        (237171.60000255704, 237_171_600_002_557),
        (1790611998.05, 1_790_611_998_049_999_872),
    ];

    /// `(base seconds, steps of f32 0.05, ns)`: the accumulations behind the table rows.
    pub(crate) const STEP_CASES: &[(f64, u32, i64)] = &[
        (0.0, 1, 50_000_001),
        (0.0, 3, 150_000_002),
        (0.0, 1340, 67_000_000_998),
        (0.0, 3432, 171_600_002_557),
        (0.0, 20000, 1_000_000_014_901),
        (237000.0, 1, 237_000_050_000_001),
        (237000.0, 3432, 237_171_600_002_557),
    ];

    #[test]
    fn nanos_matches_the_shared_table() {
        for &(secs, ns) in NANOS_CASES {
            assert_eq!(nanos(secs), ns, "nanos({secs:?})");
        }
        for &(base, k, ns) in STEP_CASES {
            let secs = (0..k).fold(base, |t, _| t + f64::from(0.05f32));
            assert_eq!(nanos(secs), ns, "{k} f32 steps from {base}");
        }
    }

    /// The f64-sum rule this replaced disagreed with acb by 1 ns: rounding `elapsed + epoch`
    /// once is not rounding each term. The integer rule sums the rounded terms.
    #[test]
    fn sim_ns_rounds_each_term_not_the_sum() {
        let mut clock = EpisodeClock::default();
        let (e_last, d_last) = (171.60000255703926, f64::from(0.05f32));
        clock.begin_episode(e_last, d_last);
        let elapsed = 67.0000009983778;
        let expected = nanos(elapsed) + nanos(e_last) + nanos(d_last);
        assert_eq!(clock.sim_ns(elapsed), expected);
        assert_eq!(expected, 67_000_000_998 + 171_600_002_557 + 50_000_001);
    }

    #[test]
    fn fmt_ns_prints_seconds_without_an_f64() {
        assert_eq!(fmt_ns(1_790_611_998_049_999_872), "1790611998.049999872");
        assert_eq!(fmt_ns(50_000_001), "0.050000001");
        assert_eq!(fmt_ns(-3), "-0.000000003");
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
        assert_eq!(clock.observe(812.5, 0.05), 812_500_000_000);
    }

    #[test]
    fn a_reload_continues_one_step_after_the_last_frame() {
        let mut carla = FakeCarla::new(0.05);
        let mut clock = EpisodeClock::default();
        let mut last_sim = 0_i64;
        for _ in 0..1000 {
            carla.tick();
            last_sim = clock.observe(carla.elapsed, carla.delta);
        }

        let out = reload_as_pause(&mut carla);
        assert_eq!(out.world, Ok("Town02"));
        let (e_last, d_last) = out.last_frame.unwrap();
        let change = clock.begin_episode(e_last, d_last);

        assert_eq!(change.old_epoch_ns, 0);
        assert_eq!(change.new_epoch_ns, nanos(e_last) + nanos(d_last));
        // New episode, elapsed back at 0: the clock lands exactly one step after the last.
        let step = nanos(carla.delta);
        assert_eq!(change.continues_at_ns, last_sim + step);
        assert_eq!(clock.sim_ns(0.0), last_sim + step);
        carla.tick();
        assert_eq!(
            clock.observe(carla.elapsed, carla.delta),
            last_sim + 2 * step
        );
    }

    #[test]
    fn epochs_accumulate_over_several_reloads() {
        let mut clock = EpisodeClock::default();
        clock.begin_episode(10.0, 0.05);
        let second = clock.begin_episode(20.0, 0.05);
        assert_eq!(second.old_epoch_ns, 10_050_000_000);
        assert_eq!(second.new_epoch_ns, 10_050_000_000 + 20_050_000_000);
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
