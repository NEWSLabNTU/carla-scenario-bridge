//! Generated protobuf types from SSv2's `simulation_interface` protocol.
//!
//! All packages are siblings under this module so cross-package references (e.g.
//! `traffic_simulator_msgs` using `super::autoware_vehicle_msgs`) resolve correctly.
//!
//! `dead_code` is allowed throughout: these modules are a transcription of the wire
//! protocol, and a message type this bridge happens not to construct is a normal state, not
//! an unused-code smell. Denying it would make `just check` fail for messages SSv2 defines
//! and we simply do not send.

#[allow(clippy::all, dead_code)]
pub mod simulation_api_schema {
    include!(concat!(env!("OUT_DIR"), "/simulation_api_schema.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod geometry_msgs {
    include!(concat!(env!("OUT_DIR"), "/geometry_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod traffic_simulator_msgs {
    include!(concat!(env!("OUT_DIR"), "/traffic_simulator_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod builtin_interfaces {
    include!(concat!(env!("OUT_DIR"), "/builtin_interfaces.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod autoware_control_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_control_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod autoware_vehicle_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_vehicle_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod rosgraph_msgs {
    include!(concat!(env!("OUT_DIR"), "/rosgraph_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod std_msgs {
    include!(concat!(env!("OUT_DIR"), "/std_msgs.rs"));
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    /// Every proto this bridge compiles, byte-identical to the SSv2 fork's copy. The two
    /// sides of the wire are built from different files; a field added to one and not the
    /// other (roadmap 015's `simulation_time`) decodes as silence, not as an error.
    #[test]
    fn protos_match_the_fork_copy() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let ours = root.join("proto");
        let fork = root.join("src/scenario_simulator_v2/simulation/simulation_interface/proto");
        assert!(
            fork.is_dir(),
            "{} missing: run `git submodule update --init --recursive`",
            fork.display()
        );
        for name in [
            "simulation_api_schema.proto",
            "geometry_msgs.proto",
            "traffic_simulator_msgs.proto",
            "builtin_interfaces.proto",
            "autoware_control_msgs.proto",
            "autoware_vehicle_msgs.proto",
            "rosgraph_msgs.proto",
            "std_msgs.proto",
        ] {
            let a = std::fs::read(ours.join(name)).unwrap();
            let b = std::fs::read(fork.join(name)).unwrap();
            assert!(
                a == b,
                "proto/{name} differs from the fork's copy in {}",
                fork.display()
            );
        }
    }
}
