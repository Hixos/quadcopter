use std::{
    path::Path,
    sync::mpsc::channel,
    thread::{sleep, spawn},
    time::Duration,
};

use anyhow::Result;
use control_system::blocks::consumers::Print;
use control_system::ControlSystemBuilder;
use control_system::{ControlSystemParameters, ParameterStore, StepResult};
use drone::{
    drivers::bmp280::{Bmp280, Bmp280Params},
    telemetry::{add_protoplotter, TelemetryServiceBuilder},
};

fn main() -> Result<()> {
    let (ts_builder_sender, ts_builder_receiver) = channel();

    let handle = spawn(|| -> Result<()> {
        run_control_system(ts_builder_sender)?;
        Ok(())
    });

    if let Ok(ts_builder) = ts_builder_receiver.recv() {
        let mut ts = ts_builder.build()?;
        ts.blocking_serve("0.0.0.0:65400".parse().unwrap())?;
    }

    handle.join().unwrap()?;
    Ok(())
}

fn run_control_system(
    ts_builder_sender: std::sync::mpsc::Sender<TelemetryServiceBuilder>,
) -> Result<()> {
    let mut store = ParameterStore::new(Path::new("drone.toml"), "cart")?;

    let mut builder = ControlSystemBuilder::default();

    builder.add_block(
        Bmp280::from_store(
            "bmp280",
            &mut store,
            Bmp280Params {
                pressure_file: "/sys/bus/iio/devices/iio:device0/in_pressure_input".to_string(),
                temperature_file: "/sys/bus/iio/devices/iio:device0/in_temp_input".to_string(),
            },
        )?,
        &[],
        &[("press", "/pressure"), ("temp", "/temperature")],
    )?;

    // builder.add_block(Print::<f64>::new("print_press"), &[("u", "/pressure")], &[])?;
    // builder.add_block(
    //     Print::<f64>::new("print_temp"),
    //     &[("u", "/temperature")],
    //     &[],
    // )?;

    // Plotters
    // let mut signals = PlotSignals::default();
    let mut ts_builder = TelemetryServiceBuilder::new(100);

    add_protoplotter::<f64>("/pressure", &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/temperature", &mut builder, &mut ts_builder)?;

    let _ = ts_builder_sender.send(ts_builder);

    // Build the control system
    let mut cs = builder.build_from_store(
        "drone",
        &mut store,
        ControlSystemParameters {
            dt: 0.01,
            max_iter: 0,
        },
    )?;

    store.save()?;

    // Execute
    while cs.step()? != StepResult::Stop {
        sleep(Duration::from_secs_f64(0.01));
    }

    println!("Control system ended");
    Ok(())
}
