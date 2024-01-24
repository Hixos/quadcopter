use std::io::Result;

fn main() -> Result<()> {
    // tonic_build::compile_protos(&["proto/telemetry.proto"], &["proto/"])?;
    tonic_build::configure()
        .build_server(true)
        .compile(
            &["proto/telemetry_rpc.proto", "proto/telemetry.proto"],
            &["proto/"],
        )?;

    Ok(())
}