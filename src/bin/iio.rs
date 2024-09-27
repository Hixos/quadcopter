use std::{thread::sleep, time::Duration};

use anyhow::{anyhow, Result};
use nix::{
    sys::time::TimeValLike,
    time::{clock_gettime, ClockId},
};

fn timestamp() -> Result<Duration> {
    let spec = clock_gettime(ClockId::CLOCK_MONOTONIC)?;
    Ok(Duration::from_nanos(spec.num_nanoseconds() as u64))
}

fn main() -> Result<()> {
    let ctx = industrial_io::Context::with_backend(industrial_io::Backend::Local)?;

    let device = ctx
        .find_device("mpu6500")
        .ok_or(anyhow!("No mpu6500 iio device found"))?;

    let ts_chan = device.find_channel("timestamp", false).unwrap();
    let accx_chan = device.find_channel("accel_x", false).unwrap();

    let trigger = ctx.find_device("mpu6500-dev0").unwrap();
    device.set_trigger(&trigger).unwrap();
    device.attr_write("sampling_frequency", 100).unwrap();
    device
        .attr_write("current_timestamp_clock", "monotonic")
        .unwrap();

    ts_chan.enable();
    accx_chan.enable();

    let mut buf = device.create_buffer(1, false)?;
    // let mut last = None;
    println!("ts,x");
    for _ in 0..10000 {
        if let Ok(n) = buf.refill() {
            let time = timestamp()?;
            for (i, (ts, x)) in buf
                .channel_iter::<u64>(&ts_chan)
                .zip(buf.channel_iter::<u16>(&accx_chan)).enumerate()
            {
                println!("{},{x}", ts as f64 / 1000000.0);
                // if let Some(last) = last {
                //     println!(
                //         "{i}, dt: {}, delay: {}",
                //         (ts - last) as f64 / 1000000.0,
                //         (time.as_nanos() as u64 - ts) as f64 / 1000000.0
                //     );
                // }
                // last = Some(ts);
            }
        }else{
            println!("refill err");
        }
    }

    Ok(())
}

fn list_iios(ctx: &industrial_io::Context) {
    for device in ctx.devices() {
        println!("{:?}", device.name());

        for attr in device.attributes() {
            println!("    {:?}", attr);
        }

        for chan in device.channels() {
            println!("    {:?}", chan.id().unwrap());
            for attr in chan.attrs() {
                println!("          {:?}", attr);
            }
        }
    }
}
