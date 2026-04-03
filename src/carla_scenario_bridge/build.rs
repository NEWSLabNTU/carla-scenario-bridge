use std::io::Result;

fn main() -> Result<()> {
    let proto_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../proto");

    let protos = &[
        format!("{proto_dir}/simulation_api_schema.proto"),
        format!("{proto_dir}/geometry_msgs.proto"),
        format!("{proto_dir}/traffic_simulator_msgs.proto"),
        format!("{proto_dir}/builtin_interfaces.proto"),
        format!("{proto_dir}/autoware_control_msgs.proto"),
        format!("{proto_dir}/autoware_vehicle_msgs.proto"),
        format!("{proto_dir}/rosgraph_msgs.proto"),
        format!("{proto_dir}/std_msgs.proto"),
    ];

    prost_build::compile_protos(protos, &[proto_dir])?;

    Ok(())
}
