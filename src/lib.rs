pub mod telemetry {
    pub mod data {
        include!(concat!(env!("OUT_DIR"), "/telemetry.data.rs"));
    }

    pub mod rpc {
        tonic::include_proto!("telemetry.rpc");
    }

    mod telemetry_server;
    mod proto_plotter;

    pub use proto_plotter::{add_protoplotter, ProtoPlotter};
    pub use telemetry_server::{TelemetryID, TelemetryService, TelemetryServiceBuilder};
}
