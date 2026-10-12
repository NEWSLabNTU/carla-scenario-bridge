//! The episode epoch, recorded in the world it belongs to (roadmap 015, "Joining mid-session").
//!
//! acb applies the epoch rule to the episode changes it watches. A vehicle side started after
//! a reload has watched none of them and would start at epoch 0, a `/clock` behind csb and
//! every other bridge by the length of the earlier episodes. acb depends on CARLA and nothing
//! else, so the epoch is put where it can read it: in CARLA. After every episode change csb
//! spawns one actor of [`BLUEPRINT`] -- an empty actor, no mesh, no collision, no physics --
//! whose `role_name` is [`role_name`]`(epoch_ns)`. `load_world` destroys it with the rest of
//! the old world, so every episode carries its own; a bridge joining mid-episode adopts it, a
//! bridge that watched the change only compares (acb `clock.rs`).
//!
//! The blueprint, the prefix and the parse rule must stay identical to acb's
//! `clock::EPOCH_MARKER_BLUEPRINT`, `EPOCH_MARKER_PREFIX` and `parse_epoch_marker`; both
//! carry the same table of cases.

/// An empty actor: nothing to render, collide with or simulate, and a settable `role_name`
/// (CARLA 0.9.16). Nothing csb or acb lists by type (`vehicle.*`, `walker.*`, `sensor.*`,
/// `traffic.*`) matches it.
pub const BLUEPRINT: &str = "util.actor.empty";

/// `role_name` prefix; the epoch in integer nanoseconds follows.
pub const PREFIX: &str = "epoch_ns:";

/// Where the marker stands: below the map, out of the way of anything that might ever look.
pub const LOCATION_Z: f32 = -100.0;

/// The marker's `role_name` for an epoch.
pub fn role_name(epoch_ns: i64) -> String {
    format!("{PREFIX}{epoch_ns}")
}

/// The epoch a `role_name` records, if it is a marker's.
pub fn parse(role_name: &str) -> Option<i64> {
    let digits = role_name.strip_prefix(PREFIX)?;
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

/// What to do with the markers a world holds, given csb's epoch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MarkerPlan {
    /// Markers recording another epoch, to destroy.
    pub stale: Vec<(u32, i64)>,
    /// Spawn a marker for csb's epoch (none records it yet).
    pub spawn: bool,
}

/// `markers`: `(actor id, epoch)` of the world's markers. Epoch 0 is never written: no
/// marker already means 0 to acb, and csb leaves a nonzero marker it disagrees with in
/// place rather than erase it with a 0 (the caller warns). Otherwise exactly one marker,
/// recording `epoch_ns`, is left.
pub fn plan(epoch_ns: i64, markers: &[(u32, i64)]) -> MarkerPlan {
    if epoch_ns == 0 {
        return MarkerPlan {
            stale: Vec::new(),
            spawn: false,
        };
    }
    let mut stale: Vec<_> = markers
        .iter()
        .copied()
        .filter(|&(_, e)| e != epoch_ns)
        .collect();
    stale.sort_unstable();
    let mut keep: Vec<_> = markers
        .iter()
        .filter(|&&(_, e)| e == epoch_ns)
        .map(|&(id, _)| id)
        .collect();
    keep.sort_unstable();
    // Duplicates of the right value: keep the newest, as acb would read it.
    if let Some((_, older)) = keep.split_last() {
        stale.extend(older.iter().map(|&id| (id, epoch_ns)));
    }
    MarkerPlan {
        spawn: keep.is_empty(),
        stale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The marker's `role_name`: shared with acb's `clock.rs` tests (`MARKER_CASES`).
    const MARKER_CASES: &[(&str, Option<i64>)] = &[
        ("epoch_ns:150000002", Some(150_000_002)),
        ("epoch_ns:0", Some(0)),
        ("epoch_ns:5819510766462", Some(5_819_510_766_462)),
        ("epoch_ns:", None),
        ("epoch_ns:-1", None),
        ("epoch_ns:+1", None),
        ("epoch_ns:1.5", None),
        ("epoch_ns: 1", None),
        ("epoch_ns:99999999999999999999", None),
        ("default", None),
        ("hero", None),
        ("EPOCH_NS:1", None),
    ];

    #[test]
    fn role_names_parse_like_acb_reads_them() {
        for &(role, epoch) in MARKER_CASES {
            assert_eq!(parse(role), epoch, "{role:?}");
        }
    }

    #[test]
    fn written_role_names_read_back() {
        for epoch in [1, 150_000_002, 5_819_510_766_462, i64::MAX] {
            assert_eq!(parse(&role_name(epoch)), Some(epoch));
        }
        assert_eq!(role_name(150_000_002), "epoch_ns:150000002");
    }

    #[test]
    fn a_fresh_world_gets_one_marker() {
        assert_eq!(
            plan(150_000_002, &[]),
            MarkerPlan {
                stale: vec![],
                spawn: true
            }
        );
    }

    #[test]
    fn a_matching_marker_is_left_alone() {
        assert_eq!(
            plan(7, &[(12, 7)]),
            MarkerPlan {
                stale: vec![],
                spawn: false
            }
        );
    }

    #[test]
    fn a_different_marker_is_replaced_and_duplicates_collapse_to_the_newest() {
        assert_eq!(
            plan(7, &[(12, 6), (30, 7), (20, 7)]),
            MarkerPlan {
                stale: vec![(12, 6), (20, 7)],
                spawn: false
            }
        );
        assert_eq!(
            plan(7, &[(12, 6)]),
            MarkerPlan {
                stale: vec![(12, 6)],
                spawn: true
            }
        );
    }

    #[test]
    fn epoch_zero_writes_and_erases_nothing() {
        assert_eq!(
            plan(0, &[(12, 6)]),
            MarkerPlan {
                stale: vec![],
                spawn: false
            }
        );
    }
}
