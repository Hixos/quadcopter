use std::{
    collections::HashMap,
    sync::mpsc::{channel, Sender},
    thread,
};

use anyhow::Result;
use drone::telemetry::{
    data::Sample,
    rpc::{
        telemetry_connector_client::TelemetryConnectorClient, StartTelemetryRequest,
        TelemetryListRequest,
    },
};
use prost::Message;
use rust_data_inspector::DataInspector;
use rust_data_inspector_signals::{PlotSampleSender, PlotSignalSample, PlotSignals};
use tokio::{net::UdpSocket, select};
use tonic::Request;

async fn async_main(signals_tx: Sender<PlotSignals>) -> Result<()> {
    let mut client = TelemetryConnectorClient::connect("http://192.168.1.24:65400").await?;

    let mut signals = PlotSignals::default();

    let telemetry_list_response = client
        .list_telemetries(Request::new(TelemetryListRequest {
            base_topic: "/".to_string(),
        }))
        .await?;

    let mut senders = HashMap::<u64, PlotSampleSender>::new();
    for telem in telemetry_list_response.get_ref().telemetries.iter() {
        let (_, sender) = signals.add_signal(&telem.name)?;
        senders.insert(telem.id, sender);
    }

    signals_tx.send(signals).expect("Could not send signals!");

    println!("Starting telemetry request!");
    select! {
        response = client
            .start_telemetry(Request::new(StartTelemetryRequest {
                ids: vec![],
                port: 65432,
            })) => {println!("Received response '{response:?}' from server")},
        _ = receive_telemetry(senders) => {}
    }

    Ok(())
}

async fn receive_telemetry(mut plot_senders: HashMap<u64, PlotSampleSender>) -> Result<()> {
    let sock = UdpSocket::bind(("0.0.0.0", 65432)).await?;
    let mut buf = vec![0; 200];

    loop {
        let (nbytes, _) = sock.recv_from(&mut buf).await?;

        if let Ok(sample) = Sample::decode(&buf[0..nbytes]) {

            if let Some(sender) = plot_senders.get_mut(&sample.id) {
                sender.send(PlotSignalSample {
                    time: sample.time,
                    value: sample.value,
                })?;
            } else {
                println!("Wrong id! {}", sample.id);
            }
        }
    }
}
fn main() -> Result<()> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;

    let (signals_tx, signals_rx) = channel();

    let handle = thread::spawn(move || rt.block_on(async_main(signals_tx)));

    let signals = signals_rx.recv()?;

    DataInspector::run_native("Remote plotter", signals).unwrap();

    handle.join().unwrap()
}
