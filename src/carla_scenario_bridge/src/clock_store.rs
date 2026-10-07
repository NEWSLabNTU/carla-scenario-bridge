//! The episode clock, kept across csb restarts.
//!
//! The epoch is the one piece of csb state that must outlive the process: a new csb that
//! starts from epoch 0 reports CARLA's raw `elapsed_seconds`, so SSv2's time goes back and
//! stays an epoch apart from acb's `/clock` (roadmap 016, gap 11). After every frame csb
//! records `(CARLA episode id, epoch, last frame)`; a new csb restores it.
//!
//! The file lives on tmpfs (`$XDG_RUNTIME_DIR`), not the log disk: it is written from the
//! frame loop, and a write that blocks in a busy disk's journal stalls the loop
//! (CLAUDE.md, "The log disk can stall a node"). It does not need to survive a reboot,
//! which takes CARLA's episode with it.

use std::path::{Path, PathBuf};

use crate::episode_clock::EpisodeClock;

/// What one csb process leaves for the next.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StoredClock {
    pub world_id: u64,
    pub epoch_ns: i64,
    pub last_frame: Option<(f64, f64)>,
}

/// How a new csb takes over the stored clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Resume {
    /// Same CARLA episode: carry on with the stored epoch.
    SameEpisode(EpisodeClock),
    /// CARLA moved to another episode (restarted, or a world was loaded) while no csb was
    /// watching: the episode rule applied to the last frame the old csb recorded.
    NewEpisode(EpisodeClock),
}

/// The clock to start with, given what was stored and the episode CARLA is in now.
pub fn resume(stored: StoredClock, current_world_id: Option<u64>) -> Resume {
    let mut clock = EpisodeClock::restored(stored.epoch_ns, stored.last_frame);
    if current_world_id == Some(stored.world_id) {
        return Resume::SameEpisode(clock);
    }
    if let Some((elapsed, delta)) = stored.last_frame {
        clock.begin_episode(elapsed, delta);
    }
    Resume::NewEpisode(clock)
}

/// `$XDG_RUNTIME_DIR/carla-scenario-bridge/clock-<host>-<port>`, or under `/tmp` without it.
pub fn path_for(host: &str, port: u16) -> PathBuf {
    let base = std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    base.join("carla-scenario-bridge")
        .join(format!("clock-{host}-{port}"))
}

/// One line: `world_id epoch_ns elapsed delta`, or `world_id epoch_ns - -` with no frame.
/// f64 `Display` is the shortest string that parses back to the same value.
fn encode(c: &StoredClock) -> String {
    match c.last_frame {
        Some((e, d)) => format!("{} {} {e} {d}\n", c.world_id, c.epoch_ns),
        None => format!("{} {} - -\n", c.world_id, c.epoch_ns),
    }
}

fn decode(text: &str) -> Option<StoredClock> {
    let mut it = text.split_whitespace();
    let world_id = it.next()?.parse().ok()?;
    let epoch_ns = it.next()?.parse().ok()?;
    let last_frame = match (it.next()?, it.next()?) {
        ("-", "-") => None,
        (e, d) => Some((e.parse().ok()?, d.parse().ok()?)),
    };
    Some(StoredClock {
        world_id,
        epoch_ns,
        last_frame,
    })
}

pub fn load(path: &Path) -> Option<StoredClock> {
    decode(&std::fs::read_to_string(path).ok()?)
}

/// Write-then-rename, so a csb killed mid-write leaves the previous record, not half a line.
pub fn save(path: &Path, clock: &StoredClock) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, encode(clock))?;
    std::fs::rename(&tmp, path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::episode_clock::nanos;

    fn stored() -> StoredClock {
        StoredClock {
            world_id: 7,
            epoch_ns: 16_686_302_864_088,
            last_frame: Some((1051.319215310, f64::from(0.05f32))),
        }
    }

    #[test]
    fn records_round_trip_exactly() {
        for c in [stored(), StoredClock { last_frame: None, ..stored() }] {
            assert_eq!(decode(&encode(&c)), Some(c));
        }
        assert_eq!(decode("garbage"), None);
        assert_eq!(decode("7 12"), None);
    }

    #[test]
    fn save_then_load() {
        let dir = std::env::temp_dir().join(format!("csb-clock-test-{}", std::process::id()));
        let path = dir.join("clock");
        save(&path, &stored()).unwrap();
        assert_eq!(load(&path), Some(stored()));
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn same_episode_keeps_the_epoch() {
        match resume(stored(), Some(7)) {
            Resume::SameEpisode(c) => {
                assert_eq!(c.epoch(), stored().epoch_ns);
                assert_eq!(c.last_frame(), stored().last_frame);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_new_episode_continues_one_step_after_the_recorded_frame() {
        let (e, d) = stored().last_frame.unwrap();
        for current in [Some(8), None] {
            match resume(stored(), current) {
                Resume::NewEpisode(c) => {
                    assert_eq!(c.epoch(), stored().epoch_ns + nanos(e) + nanos(d));
                    assert_eq!(c.sim_ns(0.0), stored().epoch_ns + nanos(e) + nanos(d));
                }
                other => panic!("{other:?}"),
            }
        }
    }
}
