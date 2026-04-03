/// Generated protobuf types from SSv2's simulation_interface protocol.
///
/// All packages are siblings under this module so cross-package references
/// (e.g., `traffic_simulator_msgs` using `super::autoware_vehicle_msgs`) resolve correctly.

#[allow(clippy::all)]
pub mod simulation_api_schema {
    include!(concat!(env!("OUT_DIR"), "/simulation_api_schema.rs"));
}

#[allow(clippy::all)]
pub mod geometry_msgs {
    include!(concat!(env!("OUT_DIR"), "/geometry_msgs.rs"));
}

#[allow(clippy::all)]
pub mod traffic_simulator_msgs {
    include!(concat!(env!("OUT_DIR"), "/traffic_simulator_msgs.rs"));
}

#[allow(clippy::all)]
pub mod builtin_interfaces {
    include!(concat!(env!("OUT_DIR"), "/builtin_interfaces.rs"));
}

#[allow(clippy::all)]
pub mod autoware_control_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_control_msgs.rs"));
}

#[allow(clippy::all)]
pub mod autoware_vehicle_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_vehicle_msgs.rs"));
}

#[allow(clippy::all)]
pub mod rosgraph_msgs {
    include!(concat!(env!("OUT_DIR"), "/rosgraph_msgs.rs"));
}

#[allow(clippy::all)]
pub mod std_msgs {
    include!(concat!(env!("OUT_DIR"), "/std_msgs.rs"));
}
