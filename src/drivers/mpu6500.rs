use crate::vector3::Vector3;
use anyhow::{anyhow, Result};
use control_system::{io::Output, Block, BlockIO, StepResult};

#[derive(BlockIO)]
pub struct Mpu6500 {
    #[blockio(block_name)]
    name: String,

    #[blockio(output)]
    dt: Output<f64>,

    #[blockio(output)]
    accel: Output<Vector3>,

    #[blockio(output)]
    angvel: Output<Vector3>,

    #[blockio(output)]
    accel_norm: Output<f64>,

    #[blockio(output)]
    angvel_norm: Output<f64>,

    buffer: industrial_io::Buffer,
    channels: Channels,

    last_ts: Option<u64>,
}

struct Channels {
    ts_chan: industrial_io::Channel,
    accel_chan: Vec<SensorChannel>,
    angvel_chan: Vec<SensorChannel>,
}

impl Channels {
    fn from_device(dev: &industrial_io::Device) -> Result<Self> {
        let ts_chan = dev
            .find_channel("timestamp", false)
            .ok_or(anyhow!("Channel '{}' not found", "timestamp"))?;

        let accx_chan = SensorChannel::from_device("accel_x", dev)?;
        let accy_chan = SensorChannel::from_device("accel_y", dev)?;
        let accz_chan = SensorChannel::from_device("accel_z", dev)?;

        let angvx_chan = SensorChannel::from_device("anglvel_x", dev)?;
        let angvy_chan = SensorChannel::from_device("anglvel_y", dev)?;
        let angvz_chan = SensorChannel::from_device("anglvel_z", dev)?;

        Ok(Self {
            ts_chan,
            accel_chan: vec![accx_chan, accy_chan, accz_chan],
            angvel_chan: vec![angvx_chan, angvy_chan, angvz_chan],
        })
    }

    fn enable(&self) {
        self.ts_chan.enable();
        self.accel_chan.iter().for_each(|c| c.chan.enable());
        self.angvel_chan.iter().for_each(|c| c.chan.enable());
    }

    fn last_timestamp(&self, buf: &industrial_io::Buffer) -> Option<u64> {
        buf.channel_iter::<u64>(&self.ts_chan).last()
    }

    fn last_accel(&self, buf: &industrial_io::Buffer) -> Option<Vector3> {
        let accel = self
            .accel_chan
            .iter()
            .map(|chan| Some(chan.calibrate(buf.channel_iter::<i16>(&chan.chan).last()?)))
            .collect::<Option<Vec<_>>>()?;

        Some(Vector3(nalgebra::Vector3::<f64>::from_column_slice(&accel)))
    }

    fn last_angvel(&self, buf: &industrial_io::Buffer) -> Option<Vector3> {
        let angvel = self
            .angvel_chan
            .iter()
            .map(|chan| Some(chan.calibrate(buf.channel_iter::<i16>(&chan.chan).last()?).to_degrees()))
            .collect::<Option<Vec<_>>>()?;

        Some(Vector3(nalgebra::Vector3::<f64>::from_column_slice(
            &angvel,
        )))
    }
}

struct SensorChannel {
    chan: industrial_io::Channel,
    bias: f64,
    scale: f64,
}

impl SensorChannel {
    fn from_device(name: &str, dev: &industrial_io::Device) -> Result<Self> {
        let chan = dev
            .find_channel(name, false)
            .ok_or(anyhow!("Channel '{name}' not found"))?;
        let bias: f64 = chan.attr_read::<f64>("calibbias")?;
        let scale: f64 = chan.attr_read::<f64>("scale")?;

        Ok(SensorChannel { chan, bias, scale })
    }

    fn calibrate(&self, raw: i16) -> f64 {
        (self.chan.convert::<i16>(raw) as f64) * self.scale
    }
}

impl Mpu6500 {
    pub fn new(
        name: &str,
        sampling_frequency: u32,
        iio_ctx: &industrial_io::Context,
    ) -> Result<Self> {
        let device = iio_ctx
            .find_device("mpu6500")
            .ok_or(anyhow!("No mpu6500 iio device found"))?;

        let channels = Channels::from_device(&device)?;

        channels.enable();

        let trigger = iio_ctx
            .find_device("mpu6500-dev0")
            .ok_or(anyhow!("No 'mpu6500-dev0' iio trigger found"))?;
        device.set_trigger(&trigger).unwrap();

        device.attr_write("sampling_frequency", sampling_frequency)?;
        device.attr_write("current_timestamp_clock", "monotonic")?;

        let buffer = device.create_buffer(1, false)?;
        Ok(Self {
            name: name.to_string(),
            dt: Output::default(),
            accel: Output::default(),
            angvel: Output::default(),
            accel_norm: Output::default(),
            angvel_norm: Output::default(),
            buffer,
            channels,
            last_ts: None,
        })
    }
}

impl Block for Mpu6500 {
    fn step(&mut self, _: control_system::StepInfo) -> control_system::Result<StepResult> {
        self.buffer
            .refill()
            .map_err(control_system::ControlSystemError::from_boxed)?;

        let ts = self.channels.last_timestamp(&self.buffer).unwrap();
        let accel = self.channels.last_accel(&self.buffer).unwrap();
        let angvel = self.channels.last_angvel(&self.buffer).unwrap();

        if let Some(last_ts) = self.last_ts {
            self.dt.set((ts - last_ts) as f64 / 1000000.0);
        } else {
            self.dt.set(0.0);
        }
        self.accel_norm.set(accel.0.norm());
        self.angvel_norm.set(angvel.0.norm());

        self.accel.set(accel);
        self.angvel.set(angvel);


        self.last_ts = Some(ts);

        Ok(StepResult::Continue)
    }
}
