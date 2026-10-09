//! The bridge's ROS node: services that configure the CARLA world (roadmap 018).
//!
//!   /carla/set_weather  csb_interfaces/srv/SetWeather
//!   /carla/get_weather  csb_interfaces/srv/GetWeather
//!
//! The node runs its own executor on its own thread, but never touches CARLA: each request
//! is handed to the ZMQ loop -- the thread that owns the CARLA client and ticks the world --
//! as a [`WorldCommand`], applied there between SSv2 requests, and answered when done. The
//! services are async, so a request waiting out a map load blocks nothing else.
//!
//! The node is named after the executable (`carla_scenario_bridge`, as the launch files
//! name it); `--ros-args` remaps and `__node:=` apply as for any ROS node. Parameters are
//! still read by [`crate::ros_args`], before the node exists.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

use csb_interfaces::msg::Weather as WeatherMsg;
use csb_interfaces::srv::get_weather::{GetWeather_Request, GetWeather_Response};
use csb_interfaces::srv::set_weather::{SetWeather_Request, SetWeather_Response};
use futures::channel::oneshot;
use rclrs::CreateBasicExecutor;

use crate::weather::{self, Weather, WeatherRequest};

pub const SET_WEATHER: &str = "/carla/set_weather";
pub const GET_WEATHER: &str = "/carla/get_weather";

/// A request for the CARLA-owning thread.
pub enum WorldCommand {
    SetWeather(WeatherRequest, oneshot::Sender<WeatherReply>),
    GetWeather(oneshot::Sender<WeatherReply>),
}

/// The answer to either weather service.
#[derive(Debug, Clone, PartialEq)]
pub struct WeatherReply {
    pub success: bool,
    pub message: String,
    pub weather: Option<Weather>,
}

impl WeatherReply {
    pub fn ok(weather: Weather, message: impl Into<String>) -> Self {
        Self {
            success: true,
            message: message.into(),
            weather: Some(weather),
        }
    }

    pub fn failed(message: impl Into<String>, weather: Option<Weather>) -> Self {
        Self {
            success: false,
            message: message.into(),
            weather,
        }
    }

    fn msg_fields(&self) -> (WeatherMsg, String) {
        match &self.weather {
            Some(w) => (
                to_msg(w),
                weather::matching_preset(w).unwrap_or_default().to_string(),
            ),
            None => (WeatherMsg::default(), String::new()),
        }
    }
}

fn to_msg(w: &Weather) -> WeatherMsg {
    let f = w.user_fields();
    WeatherMsg {
        cloudiness: f[0],
        precipitation: f[1],
        precipitation_deposits: f[2],
        wind_intensity: f[3],
        fog_density: f[4],
        fog_distance: f[5],
        wetness: f[6],
        sun_azimuth_angle: f[7],
        sun_altitude_angle: f[8],
    }
}

fn from_msg(m: &WeatherMsg) -> [f32; 9] {
    [
        m.cloudiness,
        m.precipitation,
        m.precipitation_deposits,
        m.wind_intensity,
        m.fog_density,
        m.fog_distance,
        m.wetness,
        m.sun_azimuth_angle,
        m.sun_altitude_angle,
    ]
}

/// Start the node on its own thread. Commands arrive on the returned receiver; the thread
/// ends when `shutdown` is set. An error means the node could not be created (ROS not
/// sourced, bad `--ros-args`), and nothing was started.
pub fn start(shutdown: Arc<AtomicBool>) -> eyre::Result<mpsc::Receiver<WorldCommand>> {
    let (tx, rx) = mpsc::channel();
    let (ready_tx, ready_rx) = mpsc::channel::<Result<(), String>>();
    std::thread::Builder::new()
        .name("ros_node".into())
        .spawn(move || {
            let setup = (|| -> Result<_, rclrs::RclrsError> {
                let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
                let executor = ctx.create_basic_executor();
                let node = executor.create_node(crate::ros_args::NODE_NAME)?;
                let set = {
                    let tx = tx.clone();
                    node.create_async_service::<csb_interfaces::srv::SetWeather, _>(
                        SET_WEATHER,
                        move |req: SetWeather_Request| set_weather(tx.clone(), req),
                    )?
                };
                let get = {
                    let tx = tx.clone();
                    node.create_async_service::<csb_interfaces::srv::GetWeather, _>(
                        GET_WEATHER,
                        move |_: GetWeather_Request| get_weather(tx.clone()),
                    )?
                };
                Ok((executor, node, set, get))
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
    let reply =
        match weather::decode_request(&req.preset, req.use_parameters, from_msg(&req.weather)) {
            Ok(request) => ask(tx, |r| WorldCommand::SetWeather(request, r)).await,
            Err(e) => WeatherReply::failed(e, None),
        };
    let (weather, preset) = reply.msg_fields();
    tracing::info!("{SET_WEATHER} {:?} -> {}", req.preset, reply.message);
    SetWeather_Response {
        success: reply.success,
        message: reply.message,
        weather,
        preset,
    }
}

async fn get_weather(tx: mpsc::Sender<WorldCommand>) -> GetWeather_Response {
    let reply = ask(tx, WorldCommand::GetWeather).await;
    let (weather, preset) = reply.msg_fields();
    GetWeather_Response {
        success: reply.success,
        message: reply.message,
        weather,
        preset,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msg_conversion_keeps_the_nine_fields_in_order() {
        let (_, w) = weather::preset("SoftRainSunset").unwrap();
        let m = to_msg(&w);
        assert_eq!(m.cloudiness, w.cloudiness);
        assert_eq!(m.sun_altitude_angle, w.sun_altitude_angle);
        assert_eq!(m.fog_distance, w.fog_distance);
        assert_eq!(from_msg(&m), w.user_fields());
    }

    #[test]
    fn reply_names_the_matching_preset() {
        let (_, w) = weather::preset("ClearNoon").unwrap();
        let (_, preset) = WeatherReply::ok(w, "").msg_fields();
        assert_eq!(preset, "ClearNoon");
        let custom = w.with_user_fields([1.0; 9]);
        let (m, preset) = WeatherReply::ok(custom, "").msg_fields();
        assert_eq!(preset, "");
        assert_eq!(m.wetness, 1.0);
        let (_, preset) = WeatherReply::failed("x", None).msg_fields();
        assert_eq!(preset, "");
    }
}
