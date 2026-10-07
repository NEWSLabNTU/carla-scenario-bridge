//! Per-step timing for the frame budget (docs/design/failure-and-frame-budget.md, "Frame
//! budget").
//!
//! One SSv2 step is every request from one `UpdateFrame` response to the next. Its
//! *processing* time is the handler time spent in that step minus the time spent inside
//! `world.tick()`, which is CARLA's and reported separately. Requests before a run's first
//! `UpdateFrame` (spawns, the ego's localization warm-up) belong to no step: there is no
//! previous `UpdateFrame` for them to follow.
//!
//! This module is pure bookkeeping -- it decides what to report and the caller logs it, so
//! the windowing and rate limits are testable without a subscriber.

use std::fmt;
use std::time::Duration;

/// Frames per INFO summary line: 10 s at 20 Hz.
pub const SUMMARY_WINDOW_FRAMES: usize = 200;
/// Half the 50 ms frame at 20 Hz.
pub const PROCESSING_WARN: Duration = Duration::from_millis(25);
pub const TICK_WARN: Duration = Duration::from_millis(100);

/// Which request a handler served, as far as the step accounting cares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RequestKind {
    Initialize,
    UpdateFrame,
    UpdateEntityStatus,
    Other,
}

/// One completed step.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FrameSample {
    pub frame: u64,
    pub processing: Duration,
    pub tick: Duration,
    pub update_entity_status: Duration,
    pub entities: usize,
}

/// A step over a threshold, reported in full because it is the first in its window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Warning {
    Processing(FrameSample),
    Tick(FrameSample),
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (what, limit, s) = match self {
            Warning::Processing(s) => ("processing", PROCESSING_WARN, s),
            Warning::Tick(s) => ("tick", TICK_WARN, s),
        };
        write!(
            f,
            "Frame {} {what} over {} ms: processing {}, tick {}, UpdateEntityStatus {}, \
             {} entities (further ones this window are counted in the summary)",
            s.frame,
            limit.as_millis(),
            ms(s.processing),
            ms(s.tick),
            ms(s.update_entity_status),
            s.entities
        )
    }
}

/// p50 / p95 / max of one quantity.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Distribution {
    pub p50: u64,
    pub p95: u64,
    pub max: u64,
}

impl Distribution {
    /// Nearest-rank percentiles. `None` for no values.
    pub fn of(mut values: Vec<u64>) -> Option<Self> {
        if values.is_empty() {
            return None;
        }
        values.sort_unstable();
        Some(Self {
            p50: percentile(&values, 50),
            p95: percentile(&values, 95),
            max: *values.last().unwrap(),
        })
    }
}

/// Nearest-rank percentile of sorted, non-empty `values`: the smallest value with at least
/// `pct` % of the values at or below it.
fn percentile(sorted: &[u64], pct: u64) -> u64 {
    let n = sorted.len() as u64;
    let rank = (pct * n).div_ceil(100).max(1);
    sorted[(rank - 1) as usize]
}

/// A summary over a window or a whole run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Summary {
    pub frames: usize,
    /// Nanoseconds.
    pub processing: Distribution,
    pub tick: Distribution,
    pub update_entity_status: Distribution,
    pub entities: Distribution,
    pub over_processing: usize,
    pub over_tick: usize,
}

impl Summary {
    fn of(samples: &[FrameSample]) -> Option<Self> {
        let nanos = |f: fn(&FrameSample) -> Duration| -> Vec<u64> {
            samples.iter().map(|s| f(s).as_nanos() as u64).collect()
        };
        Some(Self {
            frames: samples.len(),
            processing: Distribution::of(nanos(|s| s.processing))?,
            tick: Distribution::of(nanos(|s| s.tick))?,
            update_entity_status: Distribution::of(nanos(|s| s.update_entity_status))?,
            entities: Distribution::of(samples.iter().map(|s| s.entities as u64).collect())?,
            over_processing: samples
                .iter()
                .filter(|s| s.processing > PROCESSING_WARN)
                .count(),
            over_tick: samples.iter().filter(|s| s.tick > TICK_WARN).count(),
        })
    }
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let d = |d: Distribution| {
            format!(
                "{}/{}/{}",
                ms(Duration::from_nanos(d.p50)),
                ms(Duration::from_nanos(d.p95)),
                ms(Duration::from_nanos(d.max))
            )
        };
        write!(
            f,
            "{} frames, p50/p95/max ms: processing {}, tick {}, UpdateEntityStatus {}; \
             entities {}/{}/{}; processing > {} ms: {}, tick > {} ms: {}",
            self.frames,
            d(self.processing),
            d(self.tick),
            d(self.update_entity_status),
            self.entities.p50,
            self.entities.p95,
            self.entities.max,
            PROCESSING_WARN.as_millis(),
            self.over_processing,
            TICK_WARN.as_millis(),
            self.over_tick
        )
    }
}

fn ms(d: Duration) -> String {
    format!("{:.2}", d.as_secs_f64() * 1e3)
}

/// What one request produced for the log.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Outcome {
    /// The step this request completed (an `UpdateFrame` after the run's first).
    pub sample: Option<FrameSample>,
    pub warnings: Vec<Warning>,
    /// Set when this step filled a summary window.
    pub window_summary: Option<Summary>,
}

#[derive(Debug, Default)]
pub struct FrameStats {
    /// `UpdateFrame` requests completed in this run.
    frames: u64,
    handler: Duration,
    tick: Duration,
    update_entity_status: Duration,
    window: Vec<FrameSample>,
    warned_processing: bool,
    warned_tick: bool,
    run: Vec<FrameSample>,
}

impl FrameStats {
    pub fn new() -> Self {
        Self::default()
    }

    /// The SSv2 step a request arriving now belongs to: requests after the k-th
    /// `UpdateFrame` up to and including the (k+1)-th carry k+1.
    pub fn current_frame(&self) -> u64 {
        self.frames + 1
    }

    /// Account one handled request. `tick` is the time it spent inside `world.tick()`,
    /// `entities` the entity count after it.
    pub fn record(
        &mut self,
        kind: RequestKind,
        handler: Duration,
        tick: Duration,
        entities: usize,
    ) -> Outcome {
        let mut out = Outcome::default();
        if kind == RequestKind::Initialize {
            return out; // the caller ends the run before Initialize; nothing to count
        }
        // Before the first UpdateFrame there is no step to account to.
        if self.frames > 0 {
            self.handler += handler;
            self.tick += tick;
            if kind == RequestKind::UpdateEntityStatus {
                self.update_entity_status += handler;
            }
        }
        if kind != RequestKind::UpdateFrame {
            return out;
        }

        let started = self.frames > 0;
        self.frames += 1;
        let step = (
            std::mem::take(&mut self.handler),
            std::mem::take(&mut self.tick),
            std::mem::take(&mut self.update_entity_status),
        );
        if !started {
            return out;
        }

        let sample = FrameSample {
            frame: self.frames,
            processing: step.0.saturating_sub(step.1),
            tick: step.1,
            update_entity_status: step.2,
            entities,
        };
        out.sample = Some(sample);

        if sample.processing > PROCESSING_WARN && !self.warned_processing {
            self.warned_processing = true;
            out.warnings.push(Warning::Processing(sample));
        }
        if sample.tick > TICK_WARN && !self.warned_tick {
            self.warned_tick = true;
            out.warnings.push(Warning::Tick(sample));
        }

        self.window.push(sample);
        self.run.push(sample);
        if self.window.len() >= SUMMARY_WINDOW_FRAMES {
            out.window_summary = Summary::of(&self.window);
            self.window.clear();
            self.warned_processing = false;
            self.warned_tick = false;
        }
        out
    }

    /// Close the run: return its whole-run summary (if it had any steps) and start afresh.
    pub fn end_run(&mut self) -> Option<Summary> {
        let summary = Summary::of(&self.run);
        *self = Self::default();
        summary
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: Duration = Duration::from_millis(1);

    #[test]
    fn nearest_rank_percentiles() {
        let d = Distribution::of((1..=100).collect()).unwrap();
        assert_eq!((d.p50, d.p95, d.max), (50, 95, 100));
        let d = Distribution::of(vec![7]).unwrap();
        assert_eq!((d.p50, d.p95, d.max), (7, 7, 7));
        // Order does not matter.
        let d = Distribution::of(vec![5, 1, 4, 2, 3]).unwrap();
        assert_eq!((d.p50, d.p95, d.max), (3, 5, 5));
        assert_eq!(Distribution::of(vec![]), None);
    }

    /// One step = everything after an UpdateFrame up to the next; processing excludes tick.
    #[test]
    fn step_is_handler_time_between_update_frames_minus_tick() {
        let mut s = FrameStats::new();
        // Before the first UpdateFrame: not a step.
        s.record(RequestKind::Other, 500 * MS, Duration::ZERO, 1);
        assert_eq!(
            s.record(RequestKind::UpdateFrame, 60 * MS, 50 * MS, 1)
                .sample,
            None
        );

        assert_eq!(s.current_frame(), 2);
        s.record(RequestKind::UpdateEntityStatus, 3 * MS, Duration::ZERO, 5);
        s.record(RequestKind::Other, 1 * MS, Duration::ZERO, 5);
        let out = s.record(RequestKind::UpdateFrame, 42 * MS, 40 * MS, 5);
        let sample = out.sample.unwrap();
        assert_eq!(sample.frame, 2);
        assert_eq!(sample.processing, 6 * MS); // 3 + 1 + 42 - 40
        assert_eq!(sample.tick, 40 * MS);
        assert_eq!(sample.update_entity_status, 3 * MS);
        assert_eq!(sample.entities, 5);
        assert!(out.warnings.is_empty());
    }

    #[test]
    fn warnings_are_first_per_window_then_counted() {
        let mut s = FrameStats::new();
        s.record(RequestKind::UpdateFrame, MS, MS, 0);
        let slow = |s: &mut FrameStats| s.record(RequestKind::UpdateFrame, 230 * MS, 200 * MS, 0);

        let first = slow(&mut s);
        assert_eq!(first.warnings.len(), 2); // processing 30 ms and tick 200 ms
        assert!(matches!(first.warnings[0], Warning::Processing(_)));
        assert!(matches!(first.warnings[1], Warning::Tick(_)));
        // The rest of the window stays quiet...
        for _ in 0..(SUMMARY_WINDOW_FRAMES - 2) {
            assert!(slow(&mut s).warnings.is_empty());
        }
        // ...and the window's last frame closes it, with every slow frame counted.
        let closing = slow(&mut s);
        assert!(closing.warnings.is_empty());
        let summary = closing.window_summary.unwrap();
        assert_eq!(summary.frames, SUMMARY_WINDOW_FRAMES);
        assert_eq!(summary.over_processing, SUMMARY_WINDOW_FRAMES);
        assert_eq!(summary.over_tick, SUMMARY_WINDOW_FRAMES);
        // A new window warns again.
        assert_eq!(slow(&mut s).warnings.len(), 2);
    }

    #[test]
    fn exactly_at_the_limit_does_not_warn() {
        let mut s = FrameStats::new();
        s.record(RequestKind::UpdateFrame, MS, MS, 0);
        let out = s.record(
            RequestKind::UpdateFrame,
            PROCESSING_WARN + TICK_WARN,
            TICK_WARN,
            0,
        );
        assert!(out.warnings.is_empty());
    }

    #[test]
    fn summary_every_window_and_whole_run_at_end() {
        let mut s = FrameStats::new();
        s.record(RequestKind::UpdateFrame, MS, MS, 0);
        let mut summaries = 0;
        for i in 0..(2 * SUMMARY_WINDOW_FRAMES + 50) {
            let processing = MS * (i % 10) as u32;
            let out = s.record(RequestKind::UpdateFrame, processing + 20 * MS, 20 * MS, 3);
            summaries += out.window_summary.is_some() as usize;
        }
        assert_eq!(summaries, 2);

        let run = s.end_run().unwrap();
        assert_eq!(run.frames, 2 * SUMMARY_WINDOW_FRAMES + 50);
        assert_eq!(run.tick.p50, (20 * MS).as_nanos() as u64);
        assert_eq!(run.processing.max, (9 * MS).as_nanos() as u64);
        assert_eq!(run.entities.max, 3);
        // Ended: the next run starts from nothing.
        assert_eq!(s.end_run(), None);
        assert_eq!(s.current_frame(), 1);
    }

    #[test]
    fn a_run_with_no_steps_has_no_summary() {
        let mut s = FrameStats::new();
        s.record(RequestKind::Other, MS, Duration::ZERO, 0);
        s.record(RequestKind::UpdateFrame, MS, MS, 0);
        assert_eq!(s.end_run(), None);
    }

    #[test]
    fn tick_longer_than_handler_does_not_underflow() {
        let mut s = FrameStats::new();
        s.record(RequestKind::UpdateFrame, MS, MS, 0);
        let out = s.record(RequestKind::UpdateFrame, MS, 2 * MS, 0);
        assert_eq!(out.sample.unwrap().processing, Duration::ZERO);
    }
}
