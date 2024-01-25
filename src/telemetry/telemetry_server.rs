use std::{collections::HashMap, time::Duration};

use tokio::{
    select,
    sync::{
        mpsc::{channel, Receiver, Sender},
        oneshot, watch,
    },
    time::sleep,
};
use tokio_util::sync::CancellationToken;
use tonic::{Request, Response, Status};

use super::{
    data::Sample,
    rpc::{
        start_telemetry_reply, telemetry_connector_server::TelemetryConnector, StartTelemetryReply,
        StartTelemetryRequest, TelemetryListReply, TelemetryListRequest,
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

impl TelemetryID {
    pub fn as_u64(self) -> u64 {
        self.0
    }
}

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
        Ok(TelemetryService::new(self.telemetries))
    }
}

pub struct TelemetryService {
    telemetries: HashMap<String, TelemetryEntry>,
    stop_tx: watch::Sender<()>,
    stop_rx: watch::Receiver<()>,
}

impl TelemetryService {
    fn new(telemetries: HashMap<String, TelemetryEntry>) -> Self {
        let (stop_tx, stop_rx) = watch::channel(());
        Self {
            telemetries,
            stop_tx,
            stop_rx,
        }
    }

    pub fn stop_server(&self) {
        self.stop_tx.send(()).unwrap();
    }
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

    async fn start_telemetry(
        &self,
        request: Request<StartTelemetryRequest>,
    ) -> Result<Response<StartTelemetryReply>, Status> {
        let port = request.get_ref().port;
        println!("Received start telemetry request! port: {port}");

        let mut stop_rx = self.stop_rx.clone();

        let token = CancellationToken::new();
        let _drop_guard = token.clone().drop_guard();

        let select_task = tokio::spawn(async move {
            select! {
                    _ = Self::send_telemetry(port) => {
                        StartTelemetryReply { stop_reason: start_telemetry_reply::StopReason::TelemetryEnded as i32 }
                    },
                    _ = stop_rx.changed() => {
                        println!("Connection closed by server");
                        StartTelemetryReply { stop_reason: start_telemetry_reply::StopReason::TelemetryEnded as i32 }
                    }
                    _ = token.cancelled() => {
                        println!("Connection closed by client");
                        StartTelemetryReply { stop_reason: start_telemetry_reply::StopReason::TelemetryEnded as i32 }
                    }
            }
        });

        Ok(Response::new(select_task.await.unwrap()))
    }
}

impl TelemetryService {
    async fn send_telemetry(port: u32) {
        loop {
            println!("Sending telemetry to {port}!");
            sleep(Duration::from_secs(1)).await;
        }
    }
}
