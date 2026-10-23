//! The bridge's ROS node: services that configure the world (roadmaps 018, 051).
//!
//!   /scenario_world_bridge/set_weather  swb_interfaces/srv/SetWeather
//!   /scenario_world_bridge/get_weather  swb_interfaces/srv/GetWeather
//!
//! The weather is a preset or parameters by name, passed through to the world server, whose
//! names they are (CARLA's: its presets and `WeatherParameters` fields).
//!
//! The node runs its own executor on its own thread, but never touches CARLA: each request
//! is handed to the ZMQ loop -- the thread that owns the world client and ticks the world --
//! as a [`WorldCommand`], applied there between SSv2 requests, and answered when done. The
//! services are async, so a request waiting out a map load blocks nothing else.
//!
//! The node is named after the executable (`scenario_world_bridge`, as the launch files
//! name it); `--ros-args` remaps and `__node:=` apply as for any ROS node. Parameters are
//! still read by [`crate::ros_args`], before the node exists; the node declares the config
//! file the bridge chose as its read-only `config_file` parameter, so
//! `ros2 param get /scenario_world_bridge config_file` names the file this process loaded.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

use futures::channel::oneshot;
use rclrs::CreateBasicExecutor;
use swb_interfaces::srv::get_weather::{GetWeather_Request, GetWeather_Response};
use swb_interfaces::srv::set_weather::{SetWeather_Request, SetWeather_Response};

use crate::weather::{self, WeatherRequest};

pub const SET_WEATHER: &str = "~/set_weather";
pub const GET_WEATHER: &str = "~/get_weather";

/// A request for the world-owning thread.
pub enum WorldCommand {
    SetWeather(WeatherRequest, oneshot::Sender<WeatherReply>),
    GetWeather(oneshot::Sender<WeatherReply>),
}

pub use crate::weather::WeatherReply;

/// A reply's weather as the services' fields: the preset, the names, the values.
fn fields(reply: &WeatherReply) -> (String, Vec<String>, Vec<f64>) {
    match &reply.weather {
        Some(w) => (
            w.preset.clone(),
            w.parameters.keys().cloned().collect(),
            w.parameters.values().copied().collect(),
        ),
        None => Default::default(),
    }
}

/// Start the node on its own thread. Commands arrive on the returned receiver; the thread
/// ends when `shutdown` is set. An error means the node could not be created (ROS not
/// sourced, bad `--ros-args`), and nothing was started.
pub fn start(
    shutdown: Arc<AtomicBool>,
    config_file: &std::path::Path,
) -> eyre::Result<mpsc::Receiver<WorldCommand>> {
    let config_file: Arc<str> = config_file.display().to_string().into();
    let (tx, rx) = mpsc::channel();
    let (ready_tx, ready_rx) = mpsc::channel::<Result<(), String>>();
    std::thread::Builder::new()
        .name("ros_node".into())
        .spawn(move || {
            let setup = (|| -> Result<_, rclrs::RclrsError> {
                let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
                let executor = ctx.create_basic_executor();
                let node = executor.create_node(crate::ros_args::NODE_NAME)?;
                // The file the bridge resolved (the parameter, or the installed default it
                // fell back to), not the raw override: read-only, so `ros2 param set` cannot
                // suggest a config change the running bridge would not make.
                let config_param = node
                    .declare_parameter::<Arc<str>>("config_file")
                    .default(config_file)
                    .ignore_override()
                    .description("bridge_config.yaml this bridge loaded (set at start only)")
                    .read_only()?;
                let set = {
                    let tx = tx.clone();
                    node.create_async_service::<swb_interfaces::srv::SetWeather, _>(
                        SET_WEATHER,
                        move |req: SetWeather_Request| set_weather(tx.clone(), req),
                    )?
                };
                let get = {
                    let tx = tx.clone();
                    node.create_async_service::<swb_interfaces::srv::GetWeather, _>(
                        GET_WEATHER,
                        move |_: GetWeather_Request| get_weather(tx.clone()),
                    )?
                };
                Ok((executor, (node, config_param), set, get))
            })();
            let (mut executor, _node, _set, _get) = match setup {
                Ok(s) => {
                    let _ = ready_tx.send(Ok(()));
                    s
                }
                Err(e) => {
                    let _ = ready_tx.send(Err(e.to_string()));
                    return;
                }
            };
            while !shutdown.load(Ordering::SeqCst) {
                executor.spin(rclrs::SpinOptions::default().timeout(Duration::from_millis(200)));
            }
        })?;
    match ready_rx.recv() {
        Ok(Ok(())) => {
            tracing::info!("ROS node up: serving {SET_WEATHER}, {GET_WEATHER}");
            Ok(rx)
        }
        Ok(Err(e)) => Err(eyre::eyre!("ROS node: {e}")),
        Err(_) => Err(eyre::eyre!("ROS node thread ended before starting")),
    }
}

async fn ask(
    tx: mpsc::Sender<WorldCommand>,
    make: impl FnOnce(oneshot::Sender<WeatherReply>) -> WorldCommand,
) -> WeatherReply {
    let (reply_tx, reply_rx) = oneshot::channel();
    if tx.send(make(reply_tx)).is_err() {
        return WeatherReply::failed("the bridge is shutting down", None);
    }
    reply_rx
        .await
        .unwrap_or_else(|_| WeatherReply::failed("the bridge dropped the request", None))
}

async fn set_weather(
    tx: mpsc::Sender<WorldCommand>,
    req: SetWeather_Request,
) -> SetWeather_Response {
    let reply = match weather::decode_request(&req.preset, &req.names, &req.values) {
        Ok(request) => ask(tx, |r| WorldCommand::SetWeather(request, r)).await,
        Err(e) => WeatherReply::failed(e, None),
    };
    let (preset, names, values) = fields(&reply);
    tracing::info!(
        "{SET_WEATHER} {:?} {:?} -> {}",
        req.preset,
        req.names,
        reply.message
    );
    SetWeather_Response {
        success: reply.success,
        message: reply.message,
        preset,
        names,
        values,
    }
}

async fn get_weather(tx: mpsc::Sender<WorldCommand>) -> GetWeather_Response {
    let reply = ask(tx, WorldCommand::GetWeather).await;
    let (preset, names, values) = fields(&reply);
    GetWeather_Response {
        success: reply.success,
        message: reply.message,
        preset,
        names,
        values,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::weather::Weather;

    #[test]
    fn a_reply_becomes_the_services_fields() {
        let w = Weather {
            preset: "ClearNoon".into(),
            parameters: [
                ("cloudiness".to_string(), 5.0),
                ("wetness".to_string(), 0.0),
            ]
            .into_iter()
            .collect(),
        };
        let (preset, names, values) = fields(&WeatherReply::ok(w, ""));
        assert_eq!(preset, "ClearNoon");
        assert_eq!(names, ["cloudiness", "wetness"]);
        assert_eq!(values, [5.0, 0.0]);
        assert_eq!(fields(&WeatherReply::failed("x", None)), Default::default());
    }
}
