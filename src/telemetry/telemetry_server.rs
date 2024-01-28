use std::{
    collections::{HashMap, HashSet},
    net::{Ipv4Addr, SocketAddr},
};

use prost::Message;
use tokio::{
    net::UdpSocket,
    select,
    sync::mpsc::{channel, Receiver, Sender},
};
use tokio_util::sync::CancellationToken;
use tonic::{transport::Server, Request, Response, Status};

use super::{
    data::Sample,
    rpc::{
        start_telemetry_reply,
        telemetry_connector_server::{TelemetryConnector, TelemetryConnectorServer},
        StartTelemetryReply, StartTelemetryRequest, TelemetryListReply, TelemetryListRequest,
    },
};

use anyhow::Result;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct TelemetryEntry {
    id: TelemetryID,
    name: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TelemetryID(u64);

impl TelemetryID {
    pub fn as_u64(self) -> u64 {
        self.0
    }
}

pub struct TelemetryGrpc {
    telemetries: HashSet<TelemetryEntry>,
    subscriber_tx: Sender<TelemetrySubscriptionCmd>,
}

impl TelemetryGrpc {
    fn new(
        telemetries: HashSet<TelemetryEntry>,
        subscriber_tx: Sender<TelemetrySubscriptionCmd>,
    ) -> Self {
        Self {
            telemetries,
            subscriber_tx,
        }
    }
}

#[tonic::async_trait]
impl TelemetryConnector for TelemetryGrpc {
    async fn list_telemetries(
        &self,
        request: Request<TelemetryListRequest>,
    ) -> Result<Response<TelemetryListReply>, Status> {
        Ok(Response::new(TelemetryListReply {
            telemetries: self
                .telemetries
                .iter()
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
        if port > 65535 {
            return Ok(Response::new(StartTelemetryReply {
                stop_reason: start_telemetry_reply::StopReason::BadPort as i32,
            }));
        }
        let port = port as u16;

        let addr = SocketAddr::new(request.remote_addr().unwrap().ip(), port);

        println!("Received start telemetry request from {addr}");

        let token = CancellationToken::new();
        let _drop_guard = token.clone().drop_guard();

        if let Ok(socket) = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await {
            let _ = self
                .subscriber_tx
                .send(TelemetrySubscriptionCmd::New(addr, socket))
                .await;

            token.cancelled().await;

            println!("Connection from {addr} dropped");
            let _ = self
                .subscriber_tx
                .send(TelemetrySubscriptionCmd::Drop(addr))
                .await;

            Ok(Response::new(StartTelemetryReply {
                stop_reason: start_telemetry_reply::StopReason::TelemetryEnded as i32,
            }))
        } else {
            Ok(Response::new(StartTelemetryReply {
                stop_reason: start_telemetry_reply::StopReason::BadPort as i32,
            }))
        }
    }
}

#[derive(Debug)]
pub struct TelemetryServiceBuilder {
    counter: u64,
    telemetries: HashSet<TelemetryEntry>,
    sample_sender: Sender<Sample>,
    sample_receiver: Receiver<Sample>,
}

impl TelemetryServiceBuilder {
    pub fn new(channel_size: usize) -> Self {
        let (sample_sender, sample_receiver) = channel(channel_size);
        Self {
            counter: 0,
            telemetries: HashSet::new(),
            sample_sender,
            sample_receiver,
        }
    }

    pub fn register_signal(&mut self, signal_name: &str) -> (TelemetryID, Sender<Sample>) {
        let id = TelemetryID(self.counter);
        self.telemetries.insert(TelemetryEntry {
            id,
            name: signal_name.to_string(),
        });

        self.counter += 1;
        (id, self.sample_sender.clone())
    }

    pub fn build(self) -> anyhow::Result<TelemetryService> {
        Ok(TelemetryService {
            telemetries: self.telemetries,
            sample_receiver: self.sample_receiver,
        })
    }
}

pub struct TelemetryService {
    telemetries: HashSet<TelemetryEntry>,
    sample_receiver: Receiver<Sample>,
}

impl TelemetryService {
    pub async fn serve(&mut self, addr: std::net::SocketAddr) -> Result<()> {
        let (subs_tx, subs_rx) = channel(5);
        let grpc_service = TelemetryGrpc::new(self.telemetries.clone(), subs_tx);

        let grpc_handle = tokio::spawn(Self::serve_grpc(grpc_service, addr));
        self.send_telemetry(subs_rx).await;

        grpc_handle.await??;
        Ok(())
    }

    pub fn blocking_serve(&mut self, addr: std::net::SocketAddr) -> Result<()> {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()?;

        let _ = rt.block_on(self.serve(addr));

        Ok(())
    }

    async fn send_telemetry(&mut self, mut subscription_rx: Receiver<TelemetrySubscriptionCmd>) {
        let mut subscribers = HashMap::new();

        let mut buf: Vec<u8> = Vec::new();

        loop {
            select! {
                Some(sub_cmd) = subscription_rx.recv() => {
                    match sub_cmd {
                        TelemetrySubscriptionCmd::New(addr, socket) => {
                            subscribers.insert(addr, socket);
                        }
                        TelemetrySubscriptionCmd::Drop(addr) => {
                            subscribers.remove(&addr);
                        }
                    }
                }
                Some(sample) = self.sample_receiver.recv() => {
                    if sample.encoded_len() > buf.capacity() {
                        buf.reserve(sample.encoded_len() - buf.capacity());
                    }

                    sample.encode(&mut buf).unwrap();

                    for (dest, socket) in subscribers.iter() {
                        let _ = socket.send_to(&buf, dest).await;
                    }

                    buf.clear();
                }
            }
        }
    }

    async fn serve_grpc(grpc_service: TelemetryGrpc, addr: std::net::SocketAddr) -> Result<()> {
        Server::builder()
            .add_service(TelemetryConnectorServer::new(grpc_service))
            .serve(addr)
            .await?;

        Ok(())
    }
}

#[derive(Debug)]
enum TelemetrySubscriptionCmd {
    New(SocketAddr, UdpSocket),
    Drop(SocketAddr),
}
