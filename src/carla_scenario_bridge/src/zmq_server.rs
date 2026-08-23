use prost::Message;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::coordinator::Coordinator;
use crate::proto::simulation_api_schema::{
    self as api, simulation_request, simulation_response, SimulationRequest, SimulationResponse,
};

/// How long the bridge waits, after the ego is gone, before giving CARLA back to itself.
///
/// With no ego there is no session left to be deterministic for, so this can be short. A
/// running scenario steps at `step_time` (0.1 s by default), so ten seconds is two orders
/// of magnitude beyond a normal gap.
const IDLE_BEFORE_ASYNC_NO_EGO: Duration = Duration::from_secs(10);

/// The same, but while an ego still exists -- a last resort for a scenario that died
/// without despawning anything.
///
/// This one must not fire on a pause, because restoring async hands the world to
/// free-running physics: `fixed_delta_seconds` stops governing the step and the
/// simulation advances at whatever rate the host manages, which is precisely what
/// synchronous mode exists to prevent. SSv2 pauses routinely and legitimately -- Autoware
/// initialisation alone runs to `initialize_duration`, 120 s by default -- so this sits
/// well beyond any pause the protocol can produce. A hard-killed scenario still
/// self-heals, just in five minutes rather than ten seconds.
const IDLE_BEFORE_ASYNC_WITH_EGO: Duration = Duration::from_secs(300);

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
        let mut last_request = Instant::now();

        while !shutdown.load(Ordering::SeqCst) {
            // Poll with 100ms timeout so we can check shutdown
            let mut items = [self.socket.as_poll_item(zmq::POLLIN)];
            match zmq::poll(&mut items, 100) {
                Ok(0) => {
                    // No request for a while, but CARLA is still held in synchronous mode:
                    // the scenario ended or died without the bridge being told, so nothing
                    // is ticking and the world is frozen. Every later run then meets a dead
                    // world -- the ego spawns and cannot move, or SSv2 never gets as far as
                    // Initialize. Hand sync mode back so CARLA runs on its own again.
                    //
                    // Safe to do unprompted: if the session is merely slow, the next frame
                    // finds sync mode off and turns it back on (see decide_frame_action).
                    let limit = if self.coordinator.has_ego() {
                        IDLE_BEFORE_ASYNC_WITH_EGO
                    } else {
                        IDLE_BEFORE_ASYNC_NO_EGO
                    };
                    if self.coordinator.sync_mode_enabled() && last_request.elapsed() > limit {
                        tracing::warn!(
                            "No SSv2 request for {:?} while CARLA is in synchronous mode \
                             (ego present: {}); restoring async so the world is not left \
                             frozen",
                            limit,
                            self.coordinator.has_ego()
                        );
                        self.coordinator.restore_async_mode();
                    }
                    continue; // timeout, no message
                }
                Ok(_) => {} // message ready
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

            last_request = Instant::now();

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
                tracing::error!(
                    "Failed to decode SimulationRequest ({} bytes): {e}",
                    msg.len()
                );
                return encode_error_response(
                    peek_request_variant(msg),
                    "Failed to decode request",
                );
            }
        };

        let request_inner = match request.request {
            Some(r) => r,
            None => {
                tracing::warn!("Empty SimulationRequest (no oneof set)");
                return encode_error_response(None, "Empty request");
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

    /// Undo everything this bridge changed in CARLA: destroy its actors, unfreeze traffic
    /// lights it froze, restore async mode.
    pub fn cleanup(&mut self) {
        self.coordinator.shutdown();
    }
}

/// Recover the oneof field number from an encoded `SimulationRequest`.
///
/// Only used when full decoding failed. Protobuf encodes each field as a varint tag of
/// `(field_number << 3) | wire_type`, and `SimulationRequest` is a bare `oneof`, so the
/// first tag identifies which request was intended even when the payload after it is
/// malformed.
///
/// Returns `None` if the message is empty or the leading varint is itself unreadable.
fn peek_request_variant(msg: &[u8]) -> Option<u32> {
    // Decode a base-128 varint. Tags are small, so cap the read: field numbers here are
    // all <= 15, giving a single-byte tag, but tolerate multi-byte for robustness.
    let mut value: u64 = 0;
    for (i, &byte) in msg.iter().take(5).enumerate() {
        value |= u64::from(byte & 0x7f) << (7 * i);
        if byte & 0x80 == 0 {
            let field_number = (value >> 3) as u32;
            return (field_number != 0).then_some(field_number);
        }
    }
    None
}

/// Build a failure response, matching the request's oneof variant when it is known.
///
/// The variant matters: SSv2 calls `client.call(request).update_frame()`, and protobuf
/// returns a default-constructed message when the response holds a different variant. The
/// caller still sees `success == false`, so the failure is not lost -- but `description` is,
/// leaving the operator with a generic failure and no reason. Matching the variant keeps the
/// description attached.
///
/// `variant` is `None` when the intended request genuinely cannot be known -- an empty
/// oneof, or a message too corrupt to yield a leading tag. The response then falls back to
/// `Initialize`, which is wrong for any other request but still reads as a failure.
fn encode_error_response(variant: Option<u32>, description: &str) -> Vec<u8> {
    let result = api::Result {
        success: false,
        description: description.to_string(),
    };

    // Field numbers are from proto/simulation_api_schema.proto. SimulationRequest and
    // SimulationResponse assign the same number to each corresponding variant.
    let response = match variant {
        Some(2) => simulation_response::Response::UpdateFrame(api::UpdateFrameResponse {
            result: Some(result),
        }),
        Some(3) => {
            simulation_response::Response::SpawnVehicleEntity(api::SpawnVehicleEntityResponse {
                result: Some(result),
            })
        }
        Some(4) => simulation_response::Response::SpawnPedestrianEntity(
            api::SpawnPedestrianEntityResponse {
                result: Some(result),
            },
        ),
        Some(5) => simulation_response::Response::SpawnMiscObjectEntity(
            api::SpawnMiscObjectEntityResponse {
                result: Some(result),
            },
        ),
        Some(6) => simulation_response::Response::DespawnEntity(api::DespawnEntityResponse {
            result: Some(result),
        }),
        Some(7) => {
            simulation_response::Response::UpdateEntityStatus(api::UpdateEntityStatusResponse {
                result: Some(result),
                status: Vec::new(),
            })
        }
        Some(8) => {
            simulation_response::Response::AttachLidarSensor(api::AttachLidarSensorResponse {
                result: Some(result),
            })
        }
        Some(9) => simulation_response::Response::AttachDetectionSensor(
            api::AttachDetectionSensorResponse {
                result: Some(result),
            },
        ),
        Some(10) => simulation_response::Response::AttachOccupancyGridSensor(
            api::AttachOccupancyGridSensorResponse {
                result: Some(result),
            },
        ),
        Some(11) => {
            simulation_response::Response::UpdateTrafficLights(api::UpdateTrafficLightsResponse {
                result: Some(result),
            })
        }
        Some(13) => simulation_response::Response::AttachPseudoTrafficLightDetector(
            api::AttachPseudoTrafficLightDetectorResponse {
                result: Some(result),
            },
        ),
        Some(14) => simulation_response::Response::UpdateStepTime(api::UpdateStepTimeResponse {
            result: Some(result),
        }),
        Some(15) => simulation_response::Response::AttachImuSensor(api::AttachImuSensorResponse {
            result: Some(result),
        }),
        // Field 1 is Initialize, and it is also the documented fallback for None and for
        // any field number this build does not know.
        _ => simulation_response::Response::Initialize(api::InitializeResponse {
            result: Some(result),
        }),
    };

    SimulationResponse {
        response: Some(response),
    }
    .encode_to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode a real request, then confirm the tag peek recovers its variant. Guards the
    /// field numbers against a proto change.
    fn variant_of(request: simulation_request::Request) -> Option<u32> {
        let bytes = SimulationRequest {
            request: Some(request),
        }
        .encode_to_vec();
        peek_request_variant(&bytes)
    }

    #[test]
    fn peek_recovers_the_variant_of_a_real_request() {
        assert_eq!(
            variant_of(simulation_request::Request::Initialize(
                api::InitializeRequest::default()
            )),
            Some(1)
        );
        assert_eq!(
            variant_of(simulation_request::Request::UpdateFrame(
                api::UpdateFrameRequest::default()
            )),
            Some(2)
        );
        assert_eq!(
            variant_of(simulation_request::Request::UpdateTrafficLights(
                api::UpdateTrafficLightsRequest::default()
            )),
            Some(11)
        );
        assert_eq!(
            variant_of(simulation_request::Request::AttachImuSensor(
                api::AttachImuSensorRequest::default()
            )),
            Some(15)
        );
    }

    #[test]
    fn peek_gives_up_on_unusable_input() {
        assert_eq!(peek_request_variant(&[]), None);
        // Continuation bit set the whole way: never terminates within the cap.
        assert_eq!(peek_request_variant(&[0x80, 0x80, 0x80, 0x80, 0x80]), None);
        // Field number 0 is not valid protobuf.
        assert_eq!(peek_request_variant(&[0x00]), None);
    }

    /// The point of E1: an error for an UpdateFrame must come back as an UpdateFrame
    /// response, or SSv2's `call(req).update_frame()` default-constructs and the
    /// description is lost.
    #[test]
    fn error_response_matches_the_requested_variant() {
        let bytes = encode_error_response(Some(2), "boom");
        let decoded = SimulationResponse::decode(bytes.as_slice()).unwrap();

        match decoded.response {
            Some(simulation_response::Response::UpdateFrame(r)) => {
                let result = r.result.expect("result present");
                assert!(!result.success);
                assert_eq!(result.description, "boom");
            }
            other => panic!("expected UpdateFrame response, got {other:?}"),
        }
    }

    #[test]
    fn error_response_falls_back_to_initialize_when_variant_is_unknown() {
        let bytes = encode_error_response(None, "unknown");
        let decoded = SimulationResponse::decode(bytes.as_slice()).unwrap();

        match decoded.response {
            Some(simulation_response::Response::Initialize(r)) => {
                assert!(!r.result.expect("result present").success);
            }
            other => panic!("expected Initialize fallback, got {other:?}"),
        }
    }

    /// Every variant must produce a failure carrying the description, whichever one it is.
    #[test]
    fn every_known_variant_round_trips_as_a_failure() {
        for field in [1u32, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15] {
            let bytes = encode_error_response(Some(field), "why");
            let decoded = SimulationResponse::decode(bytes.as_slice())
                .unwrap_or_else(|e| panic!("field {field} failed to decode: {e}"));
            let response = decoded
                .response
                .unwrap_or_else(|| panic!("field {field} produced no response"));

            let result = match response {
                simulation_response::Response::Initialize(r) => r.result,
                simulation_response::Response::UpdateFrame(r) => r.result,
                simulation_response::Response::SpawnVehicleEntity(r) => r.result,
                simulation_response::Response::SpawnPedestrianEntity(r) => r.result,
                simulation_response::Response::SpawnMiscObjectEntity(r) => r.result,
                simulation_response::Response::DespawnEntity(r) => r.result,
                simulation_response::Response::UpdateEntityStatus(r) => r.result,
                simulation_response::Response::AttachLidarSensor(r) => r.result,
                simulation_response::Response::AttachDetectionSensor(r) => r.result,
                simulation_response::Response::AttachOccupancyGridSensor(r) => r.result,
                simulation_response::Response::UpdateTrafficLights(r) => r.result,
                simulation_response::Response::AttachPseudoTrafficLightDetector(r) => r.result,
                simulation_response::Response::UpdateStepTime(r) => r.result,
                simulation_response::Response::AttachImuSensor(r) => r.result,
            };

            let result = result.unwrap_or_else(|| panic!("field {field} produced no result"));
            assert!(!result.success, "field {field} should be a failure");
            assert_eq!(result.description, "why", "field {field} lost description");
        }
    }
}
