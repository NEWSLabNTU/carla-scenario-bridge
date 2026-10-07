//! ROS parameters from the command line, without a ROS client library.
//!
//! csb has no topics or services; its only ROS-facing surface is being started by `ros2 run`
//! or a launch file with parameters. Those arrive on the command line after `--ros-args`,
//! as `-p name:=value` (`ros2 run ... --ros-args -p carla_port:=2000`) or as
//! `--params-file <yaml>` (what launch_ros and play_launch write for a `<node>`'s `<param>`
//! entries). Both are parsed here; remappings, log levels and other ROS arguments are
//! accepted and ignored.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use eyre::{Result, WrapErr};

/// The node name parameters files may key their section by (besides `/**`).
pub const NODE_NAME: &str = "carla_scenario_bridge";

/// Parameter values as strings, by name. Later sources win: a `-p` after a
/// `--params-file` overrides it, as in rcl.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct RosParams(HashMap<String, String>);

impl RosParams {
    pub fn get(&self, name: &str) -> Option<&str> {
        self.0.get(name).map(String::as_str)
    }

    pub fn path(&self, name: &str) -> Option<PathBuf> {
        self.get(name).filter(|v| !v.is_empty()).map(PathBuf::from)
    }

    /// Parse the process arguments (without the program name).
    pub fn from_args<I: IntoIterator<Item = String>>(args: I) -> Result<Self> {
        let mut params = HashMap::new();
        let mut in_ros_args = false;
        let mut it = args.into_iter();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--ros-args" => in_ros_args = true,
                "--" => in_ros_args = false,
                "-p" | "--param" if in_ros_args => {
                    let kv = it.next().ok_or_else(|| eyre::eyre!("{arg} needs name:=value"))?;
                    let (k, v) = kv
                        .split_once(":=")
                        .ok_or_else(|| eyre::eyre!("{arg} {kv}: expected name:=value"))?;
                    params.insert(k.to_string(), unquote(v).to_string());
                }
                "--params-file" if in_ros_args => {
                    let file = it.next().ok_or_else(|| eyre::eyre!("--params-file needs a path"))?;
                    params.extend(params_file(Path::new(&file))?);
                }
                // Arguments with a value we do not use.
                "-r" | "--remap" | "--log-level" | "--log-config-file" | "-e" | "--enclave"
                    if in_ros_args =>
                {
                    it.next();
                }
                _ => {}
            }
        }
        Ok(Self(params))
    }
}

fn unquote(v: &str) -> &str {
    v.strip_prefix('"')
        .and_then(|v| v.strip_suffix('"'))
        .or_else(|| v.strip_prefix('\'').and_then(|v| v.strip_suffix('\'')))
        .unwrap_or(v)
}

/// The `ros__parameters` of `/**` and of this node (with or without a leading slash), in
/// that order, flattened to strings.
fn params_file(path: &Path) -> Result<HashMap<String, String>> {
    let text = std::fs::read_to_string(path).wrap_err_with(|| format!("read {}", path.display()))?;
    parse_params_yaml(&text).wrap_err_with(|| format!("parse {}", path.display()))
}

fn parse_params_yaml(text: &str) -> Result<HashMap<String, String>> {
    let doc: serde_yaml::Value = serde_yaml::from_str(text)?;
    let mut out = HashMap::new();
    let Some(map) = doc.as_mapping() else {
        return Ok(out);
    };
    let wanted = ["/**".to_string(), NODE_NAME.to_string(), format!("/{NODE_NAME}")];
    for key in &wanted {
        let Some(section) = map
            .get(serde_yaml::Value::String(key.clone()))
            .and_then(|s| s.get("ros__parameters"))
            .and_then(|p| p.as_mapping())
        else {
            continue;
        };
        for (k, v) in section {
            let Some(k) = k.as_str() else { continue };
            let v = match v {
                serde_yaml::Value::String(s) => s.clone(),
                serde_yaml::Value::Bool(b) => b.to_string(),
                serde_yaml::Value::Number(n) => n.to_string(),
                _ => continue,
            };
            out.insert(k.to_string(), v);
        }
    }
    Ok(out)
}

/// The installed default config: `<prefix>/share/carla_scenario_bridge/config/bridge_config.yaml`
/// for the first prefix in `AMENT_PREFIX_PATH` that has one.
pub fn installed_config() -> Option<PathBuf> {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").ok()?;
    prefixes
        .split(':')
        .map(|p| Path::new(p).join("share/carla_scenario_bridge/config/bridge_config.yaml"))
        .find(|p| p.is_file())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &str) -> Vec<String> {
        s.split_whitespace().map(String::from).collect()
    }

    #[test]
    fn dash_p_after_ros_args_only() {
        let p = RosParams::from_args(args(
            "-p ignored:=1 --ros-args -p carla_port:=3000 --param ssv2_port:=\"5556\" -r a:=b",
        ))
        .unwrap();
        assert_eq!(p.get("carla_port"), Some("3000"));
        assert_eq!(p.get("ssv2_port"), Some("5556"));
        assert_eq!(p.get("ignored"), None);
    }

    #[test]
    fn params_yaml_sections_and_types() {
        let p = parse_params_yaml(
            "/**:\n  ros__parameters:\n    carla_host: a\n    carla_port: 2000\n\
             /carla_scenario_bridge:\n  ros__parameters:\n    carla_host: b\n    verbose: true\n\
             other_node:\n  ros__parameters:\n    carla_host: c\n",
        )
        .unwrap();
        assert_eq!(p.get("carla_host").map(String::as_str), Some("b"));
        assert_eq!(p.get("carla_port").map(String::as_str), Some("2000"));
        assert_eq!(p.get("verbose").map(String::as_str), Some("true"));
    }

    #[test]
    fn a_later_dash_p_overrides_a_params_file() {
        let dir = std::env::temp_dir().join(format!("csb-rosargs-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("p.yaml");
        std::fs::write(&file, "/**:\n  ros__parameters:\n    carla_port: 2000\n").unwrap();
        let p = RosParams::from_args(args(&format!(
            "--ros-args --params-file {} -p carla_port:=2001",
            file.display()
        )))
        .unwrap();
        assert_eq!(p.get("carla_port"), Some("2001"));
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn malformed_param_is_an_error() {
        assert!(RosParams::from_args(args("--ros-args -p carla_port")).is_err());
        assert!(RosParams::from_args(args("--ros-args -p")).is_err());
    }
}
