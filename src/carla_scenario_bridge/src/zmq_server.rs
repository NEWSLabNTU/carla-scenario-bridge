use prost::Message;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;

use crate::coordinator::Coordinator;
use crate::frame_stats::{FrameStats, Outcome, RequestKind, SUMMARY_WINDOW_FRAMES};
use crate::proto::simulation_api_schema::{
    self as api, simulation_request, simulation_response, SimulationRequest, SimulationResponse,
};

// No idle watchdog. Until roadmap 015 step 7 the bridge handed CARLA back to async after
// 10 s without a request (300 s with an ego), so the world would not stay frozen if a
// scenario died. That free-running is what put /clock leaps and idle-gap timeouts into the
// ego's Autoware, and a REP socket cannot tell a dead SSv2 from a long pause anyway. CARLA
// now stays synchronous and paused between scenarios, csb its only ticker; a graceful
// shutdown still restores async (Coordinator::shutdown). See
// docs/design/time-and-ticking.md.

pub struct ZmqServer {
    socket: zmq::Socket,
    coordinator: Coordinator,
    /// Set when a handler panicked: why. Every request but `Initialize` is refused until the
    /// next `Initialize`, because a handler that died half-way may have left the entity map
    /// and CARLA disagreeing (docs/design/failure-and-frame-budget.md, "Panics").
    poisoned: Option<String>,
    frame_stats: FrameStats,
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
            poisoned: None,
            frame_stats: FrameStats::new(),
        })
    }

    /// Run the server loop until shutdown is signaled.
    pub fn run(&mut self, shutdown: Arc<AtomicBool>) {
        tracing::info!("ZMQ server ready, waiting for SSv2 requests...");

        while !shutdown.load(Ordering::SeqCst) {
            // Poll with 100ms timeout so we can check shutdown
            let mut items = [self.socket.as_poll_item(zmq::POLLIN)];
            match zmq::poll(&mut items, 100) {
                Ok(0) => {
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

            // Decode, dispatch, encode, send -- a panic in a handler becomes a failure
            // response instead of the end of the process.
            let variant = peek_request_variant(&msg);
            let mut poisoned = self.poisoned.take();
            let response_bytes = guarded_dispatch(&mut poisoned, variant, || self.dispatch(&msg));
            self.poisoned = poisoned;

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

        let kind = match &request_inner {
            simulation_request::Request::Initialize(_) => RequestKind::Initialize,
            simulation_request::Request::UpdateFrame(_) => RequestKind::UpdateFrame,
            simulation_request::Request::UpdateEntityStatus(_) => RequestKind::UpdateEntityStatus,
            _ => RequestKind::Other,
        };
        // Initialize starts a new run: close the previous one first, so its summary is
        // logged and this run's frames count from 1.
        if kind == RequestKind::Initialize {
            self.end_run();
        }
        let frame = match kind {
            RequestKind::Initialize => 0,
            _ => self.frame_stats.current_frame(),
        };

        // One span per handler, named after the request and carrying the SSv2 step it
        // belongs to, so every line the handler logs says which request and frame made it.
        // The subscriber does not log span enter/exit (FmtSpan::NONE): at 20 Hz that would be
        // 100+ lines a second to a disk that can stall the loop (CLAUDE.md).
        macro_rules! handle {
            ($handler:ident, $variant:ident, $req:expr) => {{
                let _span = tracing::info_span!(stringify!($handler), frame).entered();
                simulation_response::Response::$variant(self.coordinator.$handler($req))
            }};
        }

        let started = Instant::now();
        let response = match request_inner {
            simulation_request::Request::Initialize(req) => handle!(initialize, Initialize, req),
            simulation_request::Request::UpdateFrame(req) => {
                handle!(update_frame, UpdateFrame, req)
            }
            simulation_request::Request::UpdateStepTime(req) => {
                handle!(update_step_time, UpdateStepTime, req)
            }
            simulation_request::Request::SpawnVehicleEntity(req) => {
                handle!(spawn_vehicle_entity, SpawnVehicleEntity, req)
            }
            simulation_request::Request::SpawnPedestrianEntity(req) => {
                handle!(spawn_pedestrian_entity, SpawnPedestrianEntity, req)
            }
            simulation_request::Request::SpawnMiscObjectEntity(req) => {
                handle!(spawn_misc_object_entity, SpawnMiscObjectEntity, req)
            }
            simulation_request::Request::DespawnEntity(req) => {
                handle!(despawn_entity, DespawnEntity, req)
            }
            simulation_request::Request::UpdateEntityStatus(req) => {
                handle!(update_entity_status, UpdateEntityStatus, req)
            }
            simulation_request::Request::AttachLidarSensor(req) => {
                handle!(attach_lidar_sensor, AttachLidarSensor, req)
            }
            simulation_request::Request::AttachDetectionSensor(req) => {
                handle!(attach_detection_sensor, AttachDetectionSensor, req)
            }
            simulation_request::Request::AttachOccupancyGridSensor(req) => {
                handle!(attach_occupancy_grid_sensor, AttachOccupancyGridSensor, req)
            }
            simulation_request::Request::AttachImuSensor(req) => {
                handle!(attach_imu_sensor, AttachImuSensor, req)
            }
            simulation_request::Request::AttachPseudoTrafficLightDetector(req) => handle!(
                attach_pseudo_traffic_light_detector,
                AttachPseudoTrafficLightDetector,
                req
            ),
            simulation_request::Request::UpdateTrafficLights(req) => {
                handle!(update_traffic_lights, UpdateTrafficLights, req)
            }
            simulation_request::Request::UpdateEntityGoal(req) => {
                handle!(update_entity_goal, UpdateEntityGoal, req)
            }
        };
        let handler = started.elapsed();

        let tick = self.coordinator.take_tick_time();
        let entities = self.coordinator.entity_count();
        let outcome = self.frame_stats.record(kind, handler, tick, entities);
        report_frame(outcome);

        let sim_response = SimulationResponse {
            response: Some(response),
        };
        sim_response.encode_to_vec()
    }

    /// Log the finished run's frame-budget summary, if it stepped at all, and reset.
    fn end_run(&mut self) {
        if let Some(summary) = self.frame_stats.end_run() {
            tracing::info!("Frame budget, whole run: {summary}");
        }
    }

    /// Undo everything this bridge changed in CARLA: destroy its actors, unfreeze traffic
    /// lights it froze, restore async mode.
    pub fn cleanup(&mut self) {
        self.end_run();
        self.coordinator.shutdown();
    }
}

/// Log what one request produced for the frame budget: the step at DEBUG, warnings, and a
/// window summary when one closed.
fn report_frame(outcome: Outcome) {
    if let Some(s) = outcome.sample {
        tracing::debug!(
            frame = s.frame,
            processing_us = s.processing.as_micros() as u64,
            tick_us = s.tick.as_micros() as u64,
            update_entity_status_us = s.update_entity_status.as_micros() as u64,
            entities = s.entities,
            "frame timing"
        );
    }
    for warning in outcome.warnings {
        tracing::warn!("{warning}");
    }
    if let Some(summary) = outcome.window_summary {
        tracing::info!("Frame budget, last {SUMMARY_WINDOW_FRAMES} frames: {summary}");
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
/// `SimulationRequest`'s field number for `Initialize`, the one request a poisoned session
/// still serves.
const INITIALIZE_FIELD: u32 = 1;

/// Run `dispatch` so that a panic in it answers this request with a failure and poisons the
/// session instead of unwinding out of `main`.
///
/// While `poisoned` is set, every request but `Initialize` is refused without dispatching.
/// An `Initialize` that returns (whatever its result) resets the session and clears it.
fn guarded_dispatch(
    poisoned: &mut Option<String>,
    variant: Option<u32>,
    dispatch: impl FnOnce() -> Vec<u8>,
) -> Vec<u8> {
    let is_initialize = variant == Some(INITIALIZE_FIELD);
    if let (Some(reason), false) = (poisoned.as_deref(), is_initialize) {
        return encode_error_response(
            variant,
            &format!(
                "csb refused the request: an internal error ({reason}) ended this session; \
                 the next Initialize starts a new one"
            ),
        );
    }

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(dispatch)) {
        Ok(bytes) => {
            if is_initialize {
                *poisoned = None;
            }
            bytes
        }
        Err(payload) => {
            let message = panic_message(payload.as_ref());
            tracing::error!(
                "Handler panicked on request variant {variant:?}: {message}; refusing \
                 everything but Initialize until the next one"
            );
            *poisoned = Some(message.clone());
            encode_error_response(variant, &format!("csb internal error: {message}"))
        }
    }
}

fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "panic with a non-string payload".to_string())
}

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
            simulation_time_ns: 0,
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
        Some(16) => {
            simulation_response::Response::UpdateEntityGoal(api::UpdateEntityGoalResponse {
                result: Some(result),
            })
        }
        // Field 1 is Initialize, and it is also the documented fallback for None and for
        // any field number this build does not know.
        _ => simulation_response::Response::Initialize(api::InitializeResponse {
            result: Some(result),
            simulation_time_ns: 0,
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
        assert_eq!(
            variant_of(simulation_request::Request::UpdateEntityGoal(
                api::UpdateEntityGoalRequest::default()
            )),
            Some(16)
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
        for field in [1u32, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16] {
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
                simulation_response::Response::UpdateEntityGoal(r) => r.result,
            };

            let result = result.unwrap_or_else(|| panic!("field {field} produced no result"));
            assert!(!result.success, "field {field} should be a failure");
            assert_eq!(result.description, "why", "field {field} lost description");
        }
    }

    fn failure_description(bytes: &[u8]) -> Option<String> {
        let response = SimulationResponse::decode(bytes).ok()?.response?;
        let result = match response {
            simulation_response::Response::Initialize(r) => r.result,
            simulation_response::Response::UpdateFrame(r) => r.result,
            _ => None,
        }?;
        (!result.success).then_some(result.description)
    }

    fn ok_bytes() -> Vec<u8> {
        b"ok".to_vec()
    }

    #[test]
    fn a_panicking_handler_answers_with_a_failure_and_poisons_the_session() {
        let mut poisoned = None;
        let bytes = guarded_dispatch(&mut poisoned, Some(2), || panic!("entity map broke"));
        let why = failure_description(&bytes).expect("an UpdateFrame failure");
        assert!(
            why.contains("csb internal error: entity map broke"),
            "{why}"
        );
        assert_eq!(poisoned.as_deref(), Some("entity map broke"));
    }

    #[test]
    fn a_poisoned_session_refuses_everything_but_initialize() {
        let mut poisoned = Some("earlier panic".to_string());
        let mut ran = false;
        let bytes = guarded_dispatch(&mut poisoned, Some(2), || {
            ran = true;
            ok_bytes()
        });
        assert!(!ran, "a poisoned session must not dispatch UpdateFrame");
        let why = failure_description(&bytes).expect("an UpdateFrame failure");
        assert!(
            why.contains("earlier panic") && why.contains("Initialize"),
            "{why}"
        );
        assert!(poisoned.is_some());
    }

    #[test]
    fn initialize_clears_the_poison() {
        let mut poisoned = Some("earlier panic".to_string());
        assert_eq!(
            guarded_dispatch(&mut poisoned, Some(1), ok_bytes),
            ok_bytes()
        );
        assert!(poisoned.is_none());
        assert_eq!(
            guarded_dispatch(&mut poisoned, Some(2), ok_bytes),
            ok_bytes()
        );
    }

    #[test]
    fn a_panicking_initialize_keeps_the_session_poisoned() {
        let mut poisoned = Some("earlier panic".to_string());
        let bytes = guarded_dispatch(&mut poisoned, Some(1), || {
            panic!("{}", String::from("again"))
        });
        assert!(failure_description(&bytes).unwrap().contains("again"));
        assert_eq!(poisoned.as_deref(), Some("again"));
    }
}
