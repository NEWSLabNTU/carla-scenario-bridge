//! The run's CARLA contacts, written to a file for the runner (roadmap 021).
//!
//! `collision_monitor` logs what CARLA's collision sensor saw; this writes the same record
//! as one JSON file per ego run, so `physical_verdict` (csb_launch) can join it into the
//! scenario's JUnit result as a `carla_contact` testcase. SSv2 has no protocol slot for a
//! simulator-side collision, so the join has to happen after the run, where the runner
//! already merges verdicts.
//!
//! The write happens on a short-lived thread, never on the ZMQ thread: the default
//! directory is under `$HOME`, on the disk that has stalled real-time loops for seconds
//! (CLAUDE.md, "The log disk can stall a node"). Write-then-rename, so a reader never sees
//! half a report.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Bumped when a field changes meaning; `physical_verdict` refuses a schema it does not know.
pub const SCHEMA: u32 = 1;

/// Reports kept in the directory; older ones are deleted at write time.
pub const KEEP: usize = 500;

/// The value of `contact_report_dir` that turns reports off.
pub const OFF: &str = "off";

/// One other actor the ego touched during the run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReportContact {
    /// CARLA actor id; `None` for the static world.
    pub other_id: Option<u32>,
    /// SSv2 entity name, when the bridge knew the actor.
    pub name: Option<String>,
    /// CARLA blueprint id of the other actor, or empty.
    #[serde(rename = "type")]
    pub other_type: String,
    #[serde(rename = "static")]
    pub is_static: bool,
    pub first_ssv2_frame: u64,
    pub first_sim_time: f64,
    pub count: u64,
    /// Largest normal impulse of any one event, N·s.
    pub max_impulse: f64,
}

/// One ego run, from the ego's spawn to its despawn (or teardown, or CARLA restart).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactReport {
    pub schema: u32,
    /// `Initialize` count of this bridge process.
    pub episode: u64,
    /// SSv2 name of the ego.
    pub ego: String,
    /// Wall time of the ego's spawn, Unix seconds.
    pub start_unix: f64,
    /// Wall time the run ended, Unix seconds.
    pub end_unix: f64,
    /// False when the sensor was abandoned because CARLA restarted: contacts after the
    /// restart were never seen.
    pub complete: bool,
    /// In order of first contact.
    pub contacts: Vec<ReportContact>,
}

/// Where reports go, from the config/parameter value: unset is the default directory,
/// `off` is none, anything else is that path.
pub fn resolve_dir(setting: Option<&str>) -> Option<PathBuf> {
    match setting.map(str::trim) {
        Some(OFF) => None,
        Some(dir) if !dir.is_empty() => Some(PathBuf::from(dir)),
        _ => default_dir(),
    }
}

/// `$ROS_HOME/csb/contacts`, else `~/.ros/csb/contacts` -- where ROS keeps a node's files.
fn default_dir() -> Option<PathBuf> {
    let ros_home = std::env::var_os("ROS_HOME")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".ros")))?;
    Some(ros_home.join("csb").join("contacts"))
}

/// `20261011T015319.572Z_e12.json`: sorts by time, and the episode keeps two runs that
/// start in the same millisecond apart.
pub fn file_name(start_unix: f64, episode: u64) -> String {
    let ms_total = (start_unix * 1000.0).round().max(0.0) as i64;
    let (secs, ms) = (ms_total.div_euclid(1000), ms_total.rem_euclid(1000));
    let (days, sod) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    let (y, m, d) = civil_from_days(days);
    format!(
        "{y:04}{m:02}{d:02}T{:02}{:02}{:02}.{ms:03}Z_e{episode}.json",
        sod / 3600,
        sod / 60 % 60,
        sod % 60
    )
}

/// Days since 1970-01-01 to a proleptic Gregorian date (H. Hinnant's algorithm).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + i64::from(m <= 2);
    (y, m, d)
}

/// Write the report into `dir` (created if missing) and prune it to the newest `keep`
/// reports. Returns the report's path.
pub fn write(dir: &Path, report: &ContactReport, keep: usize) -> std::io::Result<PathBuf> {
    std::fs::create_dir_all(dir)?;
    let path = dir.join(file_name(report.start_unix, report.episode));
    let tmp = path.with_extension("json.tmp");
    let json = serde_json::to_string_pretty(report).map_err(std::io::Error::other)?;
    std::fs::write(&tmp, json + "\n")?;
    std::fs::rename(&tmp, &path)?;
    prune(dir, keep)?;
    Ok(path)
}

/// Delete all but the newest `keep` reports. Names sort by time, so newest is last.
fn prune(dir: &Path, keep: usize) -> std::io::Result<()> {
    let mut reports: Vec<PathBuf> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    if reports.len() <= keep {
        return Ok(());
    }
    reports.sort();
    for old in &reports[..reports.len() - keep] {
        let _ = std::fs::remove_file(old);
    }
    Ok(())
}

/// [`write`] on its own thread; the outcome is logged, never returned.
pub fn write_in_background(dir: PathBuf, report: ContactReport) {
    let spawned = std::thread::Builder::new()
        .name("contact-report".into())
        .spawn(move || match write(&dir, &report, KEEP) {
            Ok(path) => tracing::info!(
                "Contact report for ego '{}': {} contact(s) -> {}",
                report.ego,
                report.contacts.len(),
                path.display()
            ),
            Err(e) => tracing::warn!(
                "Could not write the contact report into {} ({e}); physical_verdict will \
                 find no report for this run",
                dir.display()
            ),
        });
    if let Err(e) = spawned {
        tracing::warn!("Could not start the contact-report writer ({e}); no report for this run");
    }
}

/// Wall time now, Unix seconds.
pub fn now_unix() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(tag: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("csb-contact-test-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    fn report(start: f64, episode: u64, contacts: Vec<ReportContact>) -> ContactReport {
        ContactReport {
            schema: SCHEMA,
            episode,
            ego: "ego".into(),
            start_unix: start,
            end_unix: start + 60.0,
            complete: true,
            contacts,
        }
    }

    #[test]
    fn file_names_are_utc_and_sort_by_time() {
        // 2026-10-11T01:53:19.572Z
        assert_eq!(
            file_name(1_791_683_599.572, 12),
            "20261011T015319.572Z_e12.json"
        );
        assert_eq!(file_name(0.0, 0), "19700101T000000.000Z_e0.json");
        // A rounding carry across midnight, onto a leap day.
        assert_eq!(
            file_name(951_782_399.999_6, 1),
            "20000229T000000.000Z_e1.json"
        );
        assert!(file_name(1_760_147_599.0, 9) < file_name(1_760_147_600.0, 1));
    }

    #[test]
    fn the_setting_resolves_to_a_directory_or_none() {
        assert_eq!(resolve_dir(Some("off")), None);
        assert_eq!(resolve_dir(Some(" off ")), None);
        assert_eq!(resolve_dir(Some("/x/y")), Some(PathBuf::from("/x/y")));
        // Unset and empty both mean the default, which ends in csb/contacts.
        for unset in [None, Some(""), Some("  ")] {
            let dir = resolve_dir(unset).expect("HOME is set in tests");
            assert!(dir.ends_with("csb/contacts"), "{}", dir.display());
        }
    }

    #[test]
    fn a_report_round_trips_with_the_documented_field_names() {
        let r = report(
            1_760_147_599.5,
            3,
            vec![
                ReportContact {
                    other_id: Some(233),
                    name: Some("npc_rear".into()),
                    other_type: "vehicle.tesla.model3".into(),
                    is_static: false,
                    first_ssv2_frame: 412,
                    first_sim_time: 21.05,
                    count: 37,
                    max_impulse: 1843.2,
                },
                ReportContact {
                    other_id: None,
                    name: None,
                    other_type: String::new(),
                    is_static: true,
                    first_ssv2_frame: 500,
                    first_sim_time: 25.0,
                    count: 1,
                    max_impulse: 12.0,
                },
            ],
        );
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["contacts"][0]["type"], "vehicle.tesla.model3");
        assert_eq!(json["contacts"][1]["static"], true);
        assert!(json["contacts"][1]["other_id"].is_null());
        let back: ContactReport = serde_json::from_value(json).unwrap();
        assert_eq!(back, r);
    }

    #[test]
    fn write_is_atomic_and_prunes_the_oldest() {
        let dir = scratch("prune");
        for i in 0..5u64 {
            write(&dir, &report(1_760_000_000.0 + i as f64, i, vec![]), 3).unwrap();
        }
        let mut names: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        assert_eq!(names.len(), 3, "{names:?}");
        assert!(
            names.iter().all(|n| n.ends_with(".json")),
            "no tmp left: {names:?}"
        );
        assert!(names[0].ends_with("_e2.json"), "{names:?}");
        let newest: ContactReport =
            serde_json::from_str(&std::fs::read_to_string(dir.join(&names[2])).unwrap()).unwrap();
        assert_eq!(newest.episode, 4);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
