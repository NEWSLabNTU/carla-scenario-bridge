use std::io::Result;

fn main() -> Result<()> {
    let proto_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../proto");

    prost_build::compile_protos(
        &[format!("{proto_dir}/simulation_api_schema.proto")],
        &[proto_dir],
    )?;

    Ok(())
}
