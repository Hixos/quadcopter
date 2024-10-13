use std::io::Result;
fn main() -> Result<()> {
    println!("cargo::rerun-if-changed=proto/sensors.proto");

    prost_reflect_build::Builder::new()
        .descriptor_pool("crate::DESCRIPTOR_POOL")
        .compile_protos(
            &["proto/sensors.proto", "proto/examples.proto"],
            &["proto/"],
        )?;
    Ok(())
}
