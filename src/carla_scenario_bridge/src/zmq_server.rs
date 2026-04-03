use prost::Message;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::coordinator::Coordinator;
use crate::proto::simulation_api_schema::{
    self as api, simulation_request, simulation_response, SimulationRequest, SimulationResponse,
};

pub struct ZmqServer {
    socket: zmq::Socket,
    coordinator: Coordinator,
}

impl ZmqServer {
    pub fn new(ctx: &zmq::Context, port: u16, coordinator: Coordinator) -> eyre::Result<Self> {
        let socket = ctx.socket(zmq::REP)?;
        let endpoint = format!("tcp://*:{port}");
        socket.bind(&endpoint)?;
        tracing::info!("ZMQ REP socket bound to {endpoint}");
        Ok(Self {
            socket,
            coordinator,
        })
    }

    /// Run the server loop until shutdown is signaled.
    pub fn run(&mut self, shutdown: Arc<AtomicBool>) {
        tracing::info!("ZMQ server ready, waiting for SSv2 requests...");

        while !shutdown.load(Ordering::SeqCst) {
            // Poll with 100ms timeout so we can check shutdown
            let mut items = [self.socket.as_poll_item(zmq::POLLIN)];
            match zmq::poll(&mut items, 100) {
                Ok(0) => continue,        // timeout, no message
                Ok(_) => {}               // message ready
                Err(e) => {
                    if e == zmq::Error::EINTR {
                        continue; // interrupted by signal
                    }
                    tracing::error!("zmq::poll error: {e}");
                    break;
                }
            }

            // Receive the request
            let msg = match self.socket.recv_bytes(0) {
                Ok(bytes) => bytes,
                Err(e) => {
                    tracing::error!("recv error: {e}");
                    continue;
                }
            };

            // Decode, dispatch, encode, send
            let response_bytes = self.dispatch(&msg);

            if let Err(e) = self.socket.send(&response_bytes, 0) {
                tracing::error!("send error: {e}");
            }
        }

        tracing::info!("ZMQ server shutting down");
    }

    fn dispatch(&mut self, msg: &[u8]) -> Vec<u8> {
        let request = match SimulationRequest::decode(msg) {
            Ok(r) => r,
            Err(e) => {
                tracing::error!("Failed to decode SimulationRequest: {e}");
                return encode_error_response("Failed to decode request");
            }
        };

        let request_inner = match request.request {
            Some(r) => r,
            None => {
                tracing::warn!("Empty SimulationRequest (no oneof set)");
                return encode_error_response("Empty request");
            }
        };

        let response = match request_inner {
            simulation_request::Request::Initialize(req) => {
                let resp = self.coordinator.initialize(req);
                simulation_response::Response::Initialize(resp)
            }
            simulation_request::Request::UpdateFrame(req) => {
                let resp = self.coordinator.update_frame(req);
                simulation_response::Response::UpdateFrame(resp)
            }
            simulation_request::Request::UpdateStepTime(req) => {
                let resp = self.coordinator.update_step_time(req);
                simulation_response::Response::UpdateStepTime(resp)
            }
            simulation_request::Request::SpawnVehicleEntity(req) => {
                let resp = self.coordinator.spawn_vehicle_entity(req);
                simulation_response::Response::SpawnVehicleEntity(resp)
            }
            simulation_request::Request::SpawnPedestrianEntity(req) => {
                let resp = self.coordinator.spawn_pedestrian_entity(req);
                simulation_response::Response::SpawnPedestrianEntity(resp)
            }
            simulation_request::Request::SpawnMiscObjectEntity(req) => {
                let resp = self.coordinator.spawn_misc_object_entity(req);
                simulation_response::Response::SpawnMiscObjectEntity(resp)
            }
            simulation_request::Request::DespawnEntity(req) => {
                let resp = self.coordinator.despawn_entity(req);
                simulation_response::Response::DespawnEntity(resp)
            }
            simulation_request::Request::UpdateEntityStatus(req) => {
                let resp = self.coordinator.update_entity_status(req);
                simulation_response::Response::UpdateEntityStatus(resp)
            }
            simulation_request::Request::AttachLidarSensor(req) => {
                let resp = self.coordinator.attach_lidar_sensor(req);
                simulation_response::Response::AttachLidarSensor(resp)
            }
            simulation_request::Request::AttachDetectionSensor(req) => {
                let resp = self.coordinator.attach_detection_sensor(req);
                simulation_response::Response::AttachDetectionSensor(resp)
            }
            simulation_request::Request::AttachOccupancyGridSensor(req) => {
                let resp = self.coordinator.attach_occupancy_grid_sensor(req);
                simulation_response::Response::AttachOccupancyGridSensor(resp)
            }
            simulation_request::Request::AttachImuSensor(req) => {
                let resp = self.coordinator.attach_imu_sensor(req);
                simulation_response::Response::AttachImuSensor(resp)
            }
            simulation_request::Request::AttachPseudoTrafficLightDetector(req) => {
                let resp = self.coordinator.attach_pseudo_traffic_light_detector(req);
                simulation_response::Response::AttachPseudoTrafficLightDetector(resp)
            }
            simulation_request::Request::UpdateTrafficLights(req) => {
                let resp = self.coordinator.update_traffic_lights(req);
                simulation_response::Response::UpdateTrafficLights(resp)
            }
        };

        let sim_response = SimulationResponse {
            response: Some(response),
        };
        sim_response.encode_to_vec()
    }

    /// Restore CARLA async mode on drop.
    pub fn cleanup(&mut self) {
        self.coordinator.restore_async_mode();
    }
}

fn encode_error_response(description: &str) -> Vec<u8> {
    // Return an Initialize error response as a generic error
    let resp = SimulationResponse {
        response: Some(simulation_response::Response::Initialize(
            api::InitializeResponse {
                result: Some(api::Result {
                    success: false,
                    description: description.to_string(),
                }),
            },
        )),
    };
    resp.encode_to_vec()
}
