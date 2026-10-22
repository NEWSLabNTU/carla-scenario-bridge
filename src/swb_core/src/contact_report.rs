//! The run's CARLA contacts, written to a file for the runner (roadmap 021).
//!
//! `collision_monitor` logs what CARLA's collision sensor saw; this writes the same record
//! as one JSON file per ego run, so `physical_verdict` (swb_launch) can join it into the
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

/// One contact, as the sensor callback saw it. Plain data so it can cross threads and be
/// constructed in tests.
#[derive(Debug, Clone, PartialEq)]
pub struct RawCollision {
    /// SSv2 frame: how many `UpdateFrame`s this bridge had received since `Initialize`
    /// when the event arrived; from `CollisionMonitor::drain` (CARLA) on, the SSv2 frame whose tick
    /// produced `carla_frame` (see the module docs).
    pub ssv2_frame: u64,
    /// The simulator's own frame number for the event (named for CARLA, which came first).
    pub carla_frame: u64,
    /// Simulation time of the event, in seconds.
    pub sim_time: f64,
    /// The other actor's simulator id, or `None` for a static part of the world.
    pub other_id: Option<u32>,
    /// The other actor's blueprint id (`vehicle.tesla.model3`, `walker...`), or empty.
    pub other_type: String,
    /// Magnitude of the normal impulse, N·s.
    pub impulse: f64,
    /// SSv2 name of the tracked NPC whose own sensor saw this (roadmap 032 step 5); `None`
    /// for the ego's sensor.
    pub subject: Option<String>,
}

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

/// One other actor (or the static world) a tracked NPC's own collision sensor saw it touch
/// (roadmap 032 step 5). The ego's report keeps these apart from `contacts`: they are not
/// the ego's, and `physical_verdict` judges the ego by `contacts` alone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReportNpcContact {
    /// SSv2 name of the NPC whose sensor saw the contact.
    pub npc: String,
    /// The other side, in the shape of an ego contact.
    #[serde(flatten)]
    pub contact: ReportContact,
}

/// One entity's body against the box its scenario declared (roadmap 022).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReportGeometry {
    /// SSv2 entity name.
    pub name: String,
    /// CARLA blueprint the entity was spawned as.
    pub blueprint: String,
    /// Declared length, width, height, m.
    pub declared: [f64; 3],
    /// The spawned actor's bounding box, same order, m.
    pub actual: [f64; 3],
    /// Within tolerance on every axis (0.3 m in length and width, 0.5 m in height).
    pub fits: bool,
}

/// How closely one tracked NPC followed SSv2's poses over the episode (roadmap 023).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReportTracking {
    /// SSv2 entity name.
    pub name: String,
    /// Frames measured.
    pub frames: usize,
    /// In-plane error before each step against the start-of-step pose, m.
    pub error_p95: f64,
    pub error_max: f64,
    /// Yaw error, degrees; absent for walkers.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub yaw_p95: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub yaw_max: Option<f64>,
    /// Frames on which the body was teleported back (past 0.5 m / 5 deg).
    pub snaps: u32,
    /// Frames skipped because the NPC was not in the world snapshot (no command sent).
    /// Absent in reports written before it existed.
    #[serde(default)]
    pub missed: u32,
    /// Frames driven kinematic after the body was blocked and fell back (roadmap 032).
    /// Absent in reports written before it existed.
    #[serde(default)]
    pub fallback: u32,
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
    /// Every entity spawned in the episode, in spawn order. Added with 022 under the same
    /// schema: a reader that does not know it ignores it.
    #[serde(default)]
    pub geometry: Vec<ReportGeometry>,
    /// Every tracked NPC of the episode (`npc_motion: tracked`, roadmap 023), by name. Added
    /// under the same schema; empty under `kinematic`.
    #[serde(default)]
    pub tracking: Vec<ReportTracking>,
    /// What tracked NPCs' own collision sensors saw while the ego lived (roadmap 032 step
    /// 5; `npc_contact_sensors`), by NPC in order of first contact. Added under the same
    /// schema; a reader that does not know it ignores it, so the ego's verdict is unchanged.
    #[serde(default)]
    pub npc_contacts: Vec<ReportNpcContact>,
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

/// `$ROS_HOME/swb/contacts`, else `~/.ros/swb/contacts` -- where ROS keeps a node's files.
fn default_dir() -> Option<PathBuf> {
    let ros_home = std::env::var_os("ROS_HOME")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".ros")))?;
    Some(ros_home.join("swb").join("contacts"))
}

/// `20261011T015319.572Z_e12.json`: sorts by time, and the episode keeps two runs that
/// start in the same millisecond apart.
pub fn file_name(start_unix: f64, episode: u64) -> String {
    format!("{}_e{episode}.json", utc_stamp(start_unix))
}

/// `20261011T015319.572Z`: UTC to the millisecond, sorting by time. Shared with the gap
/// trace's file names (roadmap 024).
pub fn utc_stamp(unix: f64) -> String {
    let ms_total = (unix * 1000.0).round().max(0.0) as i64;
    let (secs, ms) = (ms_total.div_euclid(1000), ms_total.rem_euclid(1000));
    let (days, sod) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    let (y, m, d) = civil_from_days(days);
    format!(
        "{y:04}{m:02}{d:02}T{:02}{:02}{:02}.{ms:03}Z",
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
            std::env::temp_dir().join(format!("swb-contact-test-{tag}-{}", std::process::id()));
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
            geometry: vec![],
            tracking: vec![],
            npc_contacts: vec![],
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
        // Unset and empty both mean the default, which ends in swb/contacts.
        for unset in [None, Some(""), Some("  ")] {
            let dir = resolve_dir(unset).expect("HOME is set in tests");
            assert!(dir.ends_with("swb/contacts"), "{}", dir.display());
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

    /// Roadmap 032 step 5: what tracked NPCs' own sensors saw is a list of its own, so
    /// `physical_verdict`, which reads `contacts`, judges the ego's contacts and nothing else.
    #[test]
    fn npc_contacts_are_a_separate_optional_list() {
        let old = r#"{"schema":1,"episode":1,"ego":"ego","start_unix":1.0,"end_unix":2.0,
                      "complete":true,"contacts":[]}"#;
        let back: ContactReport = serde_json::from_str(old).unwrap();
        assert!(back.npc_contacts.is_empty());
        let mut r = report(1.0, 1, vec![]);
        r.npc_contacts.push(ReportNpcContact {
            npc: "npc_05".into(),
            contact: ReportContact {
                other_id: None,
                name: None,
                other_type: "static.pole".into(),
                is_static: true,
                first_ssv2_frame: 25,
                first_sim_time: 1.25,
                count: 3,
                max_impulse: 4100.0,
            },
        });
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["contacts"], serde_json::json!([]));
        // Flat: the NPC's name beside the same fields an ego contact has.
        assert_eq!(json["npc_contacts"][0]["npc"], "npc_05");
        assert_eq!(json["npc_contacts"][0]["static"], true);
        assert_eq!(json["npc_contacts"][0]["type"], "static.pole");
        let back: ContactReport = serde_json::from_value(json).unwrap();
        assert_eq!(back, r);
    }

    #[test]
    fn geometry_is_optional_in_schema_1() {
        // A 021 report, written before the field existed, still reads.
        let old = r#"{"schema":1,"episode":1,"ego":"ego","start_unix":1.0,"end_unix":2.0,
                      "complete":true,"contacts":[]}"#;
        let back: ContactReport = serde_json::from_str(old).unwrap();
        assert!(back.geometry.is_empty());
        let mut r = report(1.0, 1, vec![]);
        r.geometry.push(ReportGeometry {
            name: "truck".into(),
            blueprint: "vehicle.carlamotors.firetruck".into(),
            declared: [12.0, 2.5, 3.5],
            actual: [8.468, 2.891, 3.827],
            fits: false,
        });
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["geometry"][0]["fits"], false);
        assert_eq!(json["geometry"][0]["declared"][0], 12.0);
        assert!(back.tracking.is_empty());
    }

    #[test]
    fn tracking_rows_omit_yaw_for_walkers() {
        let mut r = report(1.0, 1, vec![]);
        r.tracking.push(ReportTracking {
            name: "pedestrian".into(),
            frames: 100,
            error_p95: 0.01,
            error_max: 0.2,
            yaw_p95: None,
            yaw_max: None,
            snaps: 0,
            missed: 2,
            fallback: 30,
        });
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["tracking"][0]["snaps"], 0);
        assert_eq!(json["tracking"][0]["missed"], 2);
        assert!(json["tracking"][0].get("yaw_p95").is_none());
        let back: ContactReport = serde_json::from_value(json.clone()).unwrap();
        assert_eq!(back.tracking[0].yaw_max, None);
        assert_eq!(back.tracking[0].missed, 2);
        assert_eq!(back.tracking[0].fallback, 30);
        // A report written before `missed` existed still parses, as 0.
        let mut old = json;
        old["tracking"][0].as_object_mut().unwrap().remove("missed");
        // ... and so does one written before `fallback` (roadmap 032).
        old["tracking"][0]
            .as_object_mut()
            .unwrap()
            .remove("fallback");
        let back: ContactReport = serde_json::from_value(old).unwrap();
        assert_eq!(back.tracking[0].missed, 0);
        assert_eq!(back.tracking[0].fallback, 0);
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
