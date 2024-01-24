use std::collections::HashMap;

use tokio::sync::mpsc::{channel, Receiver, Sender};
use tonic::{transport::Server, Request, Response, Status};

use crate::telemetry::rpc::telemetry_connector_server::TelemetryConnectorServer;

use super::{
    data::Sample,
    rpc::{
        telemetry_connector_server::TelemetryConnector, TelemetryListReply, TelemetryListRequest,
    },
};

#[derive(Debug, Default)]
pub struct TelemetryServiceBuilder {
    counter: u64,
    telemetries: HashMap<String, TelemetryEntry>,
}

#[derive(Debug)]
struct TelemetryEntry {
    id: TelemetryID,
    name: String,
    receiver: Receiver<Sample>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TelemetryID(u64);

impl TelemetryServiceBuilder {
    pub fn register_signal(
        &mut self,
        signal_name: &str,
        channel_size: usize,
    ) -> (TelemetryID, Sender<Sample>) {
        let (sender, receiver) = channel(channel_size);

        let id = TelemetryID(self.counter);
        self.telemetries.insert(
            signal_name.to_string(),
            TelemetryEntry {
                id,
                name: signal_name.to_string(),
                receiver,
            },
        );

        self.counter += 1;
        (id, sender)
    }

    pub fn build(self) -> anyhow::Result<TelemetryService> {
        Ok(TelemetryService {
            telemetries: self.telemetries,
        })
    }
}

pub struct TelemetryService {
    telemetries: HashMap<String, TelemetryEntry>,
}

#[tonic::async_trait]
impl TelemetryConnector for TelemetryService {
    async fn list_telemetries(
        &self,
        request: Request<TelemetryListRequest>,
    ) -> Result<Response<TelemetryListReply>, Status> {
        Ok(Response::new(TelemetryListReply {
            telemetries: self
                .telemetries
                .values()
                .filter(|t| t.name.starts_with(&request.get_ref().base_topic))
                .map(|t| super::rpc::telemetry_list_reply::Telemetry {
                    id: t.id.0,
                    name: t.name.clone(),
                })
                .collect(),
        }))
    }
}
