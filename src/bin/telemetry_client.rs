use std::{
    net::{Ipv4Addr, SocketAddrV4, UdpSocket}, str::FromStr, thread, time::Duration
};

use anyhow::Result;
use bytes::{Bytes, BytesMut};
use drone::telemetry::{
    data::Sample,
    rpc::{
        telemetry_connector_client::TelemetryConnectorClient, StartTelemetryRequest,
        TelemetryListRequest,
    },
};
use prost::Message;
use tokio::{select, sync::oneshot, time::sleep};
use tonic::Request;

async fn async_main() -> Result<()> {
    let mut client = TelemetryConnectorClient::connect("http://127.0.0.1:65400").await?;

    let telemetry_list_response = client
        .list_telemetries(Request::new(TelemetryListRequest {
            base_topic: "/".to_string(),
        }))
        .await?;
    println!("{:?}", telemetry_list_response);

    select! {
        response = client
        .start_telemetry(Request::new(StartTelemetryRequest {
            ids: vec![],
            port: 1234,
        }))
        => {
            if let Ok(response) = &response {
                println!(
                    "Telemetry terminated by server: {}",
                    response.get_ref().stop_reason().as_str_name()
                );
            }
            response.map(|_| ())
       }
       _ = sleep(Duration::from_secs(20)) => {Ok(())}
    }?;

    Ok(())
}

fn main() -> Result<()> {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;

    let handle = thread::spawn(move || rt.block_on(async_main()));

    handle.join().unwrap()

    // let mut sample = Sample::default();

    // let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
    // let dest_addr = SocketAddrV4::new(Ipv4Addr::from_str("127.0.0.1")?, 65500);
    // socket.set_nonblocking(true)?;

    // let mut i = 0;
    // let mut buf: Vec<u8> = Vec::new();

    // loop {
    //     sample.id = i;
    //     sample.time = i as f64;
    //     sample.value = i as f64 * 10.0;

    //     let len = sample.encoded_len();

    //     buf.resize(len, 0);

    //     sample.encode(&mut buf)?;

    //     socket.send_to(buf.as_slice(), dest_addr)?;
    //     sleep(Duration::from_millis(100));

    //     i += 1;
    // }
}
