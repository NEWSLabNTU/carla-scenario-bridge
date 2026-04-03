/// ZMQ REP server implementing SSv2's simulation_interface protocol.
///
/// Binds to `tcp://*:PORT` and handles 14 protobuf request types,
/// dispatching each to the appropriate handler in the Coordinator.
///
/// Reference: scenario_simulator_v2/simulation/simulation_interface/
///   include/simulation_interface/zmq_multi_server.hpp

// TODO(Phase 1): ZMQ REP socket binding
// TODO(Phase 1): Protobuf deserialization (SimulationRequest oneof)
// TODO(Phase 1): Request dispatch to Coordinator methods
// TODO(Phase 1): Response serialization and send
