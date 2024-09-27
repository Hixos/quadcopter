use std::{
    path::Path,
    sync::mpsc::channel,
    thread::{sleep, spawn},
    time::Duration,
};

use anyhow::Result;
use control_system::ControlSystemBuilder;
use control_system::{ControlSystemParameters, ParameterStore, StepResult};
use drone::{
    drivers::{
        bmp280::{Bmp280, Bmp280Params}, lis3mdl::Lis3mdl, lsm6dsv::Lsm6dsv, mpu6500::Mpu6500, mpu9250::{Mpu9250, Mpu9250Params}
    },
    telemetry::{add_protoplotter, TelemetryServiceBuilder},
    vector3::Vector3,
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
    let ctx = industrial_io::Context::with_backend(industrial_io::Backend::Local)?;

    // builder.add_block(
    //     Bmp280::from_store(
    //         "bmp280",
    //         &mut store,
    //         Bmp280Params {
    //             pressure_file: "/sys/bus/iio/devices/iio:device3/in_pressure_input".to_string(),
    //             temperature_file: "/sys/bus/iio/devices/iio:device3/in_temp_input".to_string(),
    //         },
    //     )?,
    //     &[],
    //     &[
    //         ("press", "/bmp280/pressure"),
    //         ("temp", "/bmp280/temperature"),
    //     ],
    // )?;

    builder.add_block(
        Lis3mdl::new("lis3mdl", 80, &ctx)?,
        &[],
        &[
            ("mag", "/lis3mdl/mag"),
            ("mag_norm", "/lis3mdl/mag_norm"),
            ("dt", "/lis3mdl/dt"),
        ],
    )?;

    // builder.add_block(
    //     Lsm6dsv::new("lsm6dsv", 60, &ctx)?,
    //     &[],
    //     &[
    //         ("accel", "/lsm6dsv16x/accel"),
    //         ("accel_norm", "/lsm6dsv16x/accel_norm"),
    //         ("angvel", "/lsm6dsv16x/angvel"),
    //         ("angvel_norm", "/lsm6dsv16x/angvel_norm"),
    //         ("dt_accel", "/lsm6dsv16x/dt_accel"),
    //         ("dt_gyro", "/lsm6dsv16x/dt_gyro"),
    //     ],
    // )?;

    // builder.add_block(
    //     Mpu9250::from_store("mpu65002", &mut store, Mpu9250Params { device: "iio:device0".to_string() })?,
    //     &[],
    //     &[
    //         ("acc", "/mpu65002/accel"),
    //         ("gyro", "/mpu65002/angvel"),
    //     ],
    // )?;

    // builder.add_block(Print::<f64>::new("print_press"), &[("u", "/pressure")], &[])?;
    // builder.add_block(
    //     Print::<f64>::new("print_temp"),
    //     &[("u", "/temperature")],
    //     &[],
    // )?;

    // Plotters
    // let mut signals = PlotSignals::default();
    let mut ts_builder = TelemetryServiceBuilder::new(100);

    // add_protoplotter::<f64>("/bmp280/pressure", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<f64>("/bmp280/temperature", &mut builder, &mut ts_builder)?;

    add_protoplotter::<Vector3>("/lis3mdl/mag", &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/lis3mdl/mag_norm", &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/lis3mdl/dt", &mut builder, &mut ts_builder)?;

    // add_protoplotter::<Vector3>("/lsm6dsv16x/accel", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<f64>("/lsm6dsv16x/accel_norm", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<Vector3>("/lsm6dsv16x/angvel", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<f64>("/lsm6dsv16x/angvel_norm", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<f64>("/lsm6dsv16x/dt_accel", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<f64>("/lsm6dsv16x/dt_gyro", &mut builder, &mut ts_builder)?;

    // add_protoplotter::<Vector3>("/mpu65002/angvel", &mut builder, &mut ts_builder)?;
    // add_protoplotter::<Vector3>("/mpu65002/accel", &mut builder, &mut ts_builder)?;
    
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
        // sleep(Duration::from_secs_f64(0.01));
    }

    println!("Control system ended");
    Ok(())
}
