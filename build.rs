use std::io::Result;

fn main() -> Result<()> {
    tonic_build::configure()
        .build_server(true)
        .compile(
            &["proto/telemetry_rpc.proto", "proto/telemetry.proto"],
            &["proto/"],
        )?;

    Ok(())
}